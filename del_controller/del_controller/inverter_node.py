#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelInverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_inverter')
        # Subscribe to the *raw* commands:
        self.sub = self.create_subscription(
            Twist, 
            'cmd_vel_raw', 
            self.twist_callback, 
            10
        )
        # Republish on the *real* cmd_vel:
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def twist_callback(self, msg: Twist):
        inverted = Twist()
        # copy forward/backward exactly
        inverted.linear.x  = msg.linear.x
        inverted.linear.y  = msg.linear.y
        inverted.linear.z  = msg.linear.z
        # copy roll/pitch exactly
        inverted.angular.x = msg.angular.x
        inverted.angular.y = msg.angular.y
        # **only flip yaw** (left/right)
        inverted.angular.z = -msg.angular.z
        self.pub.publish(inverted)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelInverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
