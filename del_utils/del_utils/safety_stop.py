#!/usr/bin/env python3

import math
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException
from geometry_msgs.msg import TransformStamped

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop_node')

        # Parameters
        self.declare_parameter('danger_distance', 0.2)
        self.declare_parameter('warning_distance', 0.6)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safety_stop_topic', '/safety_stop')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('zone_topic', '/zone')

        self.danger_distance = self.get_parameter('danger_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.safety_stop_topic = self.get_parameter('safety_stop_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.zone_topic = self.get_parameter('zone_topic').value

        # TF buffer & listener (to transform into odom)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS interfaces
        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.zone_pub = self.create_publisher(MarkerArray, self.zone_topic, 10)

        # State
        self.state = State.FREE

        # Build static markers
        self.zones = MarkerArray()
        warning = Marker()
        warning.id = 0
        warning.type = Marker.CYLINDER
        warning.action = Marker.ADD
        warning.scale.x = self.warning_distance * 2
        warning.scale.y = self.warning_distance * 2
        warning.scale.z = 0.001
        warning.color.r = 1.0
        warning.color.g = 0.984
        warning.color.b = 0.0
        warning.color.a = 0.5

        danger = Marker()
        danger.id = 1
        danger.type = Marker.CYLINDER
        danger.action = Marker.ADD
        danger.scale.x = self.danger_distance * 2
        danger.scale.y = self.danger_distance * 2
        danger.scale.z = 0.001
        danger.color.r = 1.0
        danger.color.g = 0.0
        danger.color.b = 0.0
        danger.color.a = 0.3

        self.zones.markers = [warning, danger]

    def laser_callback(self, msg: LaserScan):
        # Determine current state
        current_state = State.FREE
        for r in msg.ranges:
            if not math.isinf(r) and r <= self.warning_distance:
                current_state = State.WARNING
                if r <= self.danger_distance:
                    current_state = State.DANGER
                    break

        # Always publish velocity & safety stop each cycle
        stop_msg = Bool()
        twist = Twist()
        if current_state == State.WARNING:
            stop_msg.data = False
            twist.linear.x = 0.1  # slow
        elif current_state == State.DANGER:
            stop_msg.data = True
            twist.linear.x = 0.0  # stop
        else:
            stop_msg.data = False
            twist.linear.x = 0.5  # normal
        self.cmd_vel_pub.publish(twist)
        self.safety_stop_pub.publish(stop_msg)

        # Transform pose into odom frame
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'odom',              # target frame
                msg.header.frame_id, # source frame
                rclpy.time.Time()    # time
            )
            frame_id = 'odom'
        except LookupException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            # fallback: use sensor frame
            frame_id = msg.header.frame_id
            trans = None

        # Update marker headers and poses
        now = self.get_clock().now().to_msg()
        for zone in self.zones.markers:
            zone.header.frame_id = frame_id
            zone.header.stamp = now
            if trans:
                zone.pose.position.x = trans.transform.translation.x
                zone.pose.position.y = trans.transform.translation.y
                zone.pose.position.z = trans.transform.translation.z + (0.01 if zone.id == 0 else 0.02)
                zone.pose.orientation = trans.transform.rotation
            # adjust alpha
            if current_state == State.FREE:
                zone.color.a = 0.5
            elif current_state == State.WARNING and zone.id == 0:
                zone.color.a = 1.0
            elif current_state == State.DANGER:
                zone.color.a = 1.0

        self.zone_pub.publish(self.zones)
        self.state = current_state


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()