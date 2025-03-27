#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # New import

import math

def quaternion_to_euler_angle(w, x, y, z):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).
    Returns the yaw in degrees.
    """
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = math.atan2(t3, t4)

    # Convert yaw to degrees
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')

        # Configure QoS profile for best-effort reliability
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/robot_1/interface/mavros/local_position/odom',  # Change this to your odometry topic name if different
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Extract position
        position = msg.pose.pose.position
        x, y, z = position.x, position.y, position.z

        # Extract orientation (quaternion)
        orientation = msg.pose.pose.orientation
        quat_x, quat_y, quat_z, quat_w = orientation.x, orientation.y, orientation.z, orientation.w

        # Convert quaternion to Euler angles and get yaw in degrees
        roll, pitch, yaw = quaternion_to_euler_angle(quat_w, quat_x, quat_y, quat_z)

        # Print position and yaw
        self.get_logger().info(f"Position: x={x}, y={y}, z={z}")
        self.get_logger().info(f"Rotation: roll={roll}, pitch={pitch}, yaw={yaw} degrees")

def main(args=None):
    rclpy.init(args=args)
    node = OdometrySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
