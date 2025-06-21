#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_=self.create_publisher(Twist, "/cmd_vel",10)
        self.timer_=self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        msg=Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.5
        self.cmd_vel_pub_.publish(msg)    



def main(args=None):
    rclpy.init(args=args)
    node= DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()