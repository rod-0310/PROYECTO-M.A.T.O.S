#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from tf2_ros import TransformBroadcaster
import math

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_odom)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # Ángulo en radianes

    def publish_odom(self):
        # Simula movimiento en círculo
        v = 0.00001   # Velocidad lineal
        w = 0.001   # Velocidad angular (radianes por ciclo)

        # Actualiza posición
        self.theta += w
        self.x += v * math.cos(self.theta)
        self.y += v * math.sin(self.theta)

        # Mensaje Odometry
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "camera_depth_frame"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Calcula el cuaternión para el ángulo theta
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation = q
        odom.twist.twist = Twist()
        self.odom_pub.publish(odom)

        # Publica la transformación TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "camera_depth_frame"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdom()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
