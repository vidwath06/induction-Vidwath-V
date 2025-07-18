#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import time

class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.forward_timer = self.create_timer(0.1, self.keep_moving_forward)

        self.move_forward = True
        self.processing = False
        self.count = 0
        self.max_tags = 5

        self.get_logger().info("Aruco Navigator ready and running!")

    def keep_moving_forward(self):
        if self.move_forward and not self.processing and self.count < self.max_tags:
            twist = Twist()
            twist.linear.x = 0.08  
            self.cmd_pub.publish(twist)

    def image_callback(self, msg):
        if self.processing or self.count >= self.max_tags:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
            parameters = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(aruco_dict, parameters)

            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                for tag_id in ids.flatten():
                    if tag_id in [0, 1] and not self.processing:
                        self.move_forward = False
                        self.processing = True
                        self.get_logger().info(f"[INFO] Detected tag ID: {tag_id}")
                        self.handle_tag(tag_id)
                        self.count += 1
                        self.processing = False
                        self.move_forward = True
                        time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    def handle_tag(self, tag_id):
        twist = Twist()
        turn_speed = 0.5         
        forward_speed = 0.25     
        forward_distance = 0.3  
        turn_time = 3.1         

        # Turn
        twist.angular.z = -turn_speed if tag_id == 0 else turn_speed
        self.get_logger().info("[ACTION] Turning...")
        start = time.time()
        while time.time() - start < turn_time:
            self.cmd_pub.publish(twist)

        # Move forward
        twist.angular.z = 0.0
        twist.linear.x = forward_speed
        forward_time = forward_distance / forward_speed 

        self.get_logger().info("[ACTION] Moving forward...")
        start = time.time()
        while time.time() - start < forward_time:
            self.cmd_pub.publish(twist)

        # Stop
        self.cmd_pub.publish(Twist())
        self.get_logger().info("[ACTION] Completed 90° turn + forward.\n")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
