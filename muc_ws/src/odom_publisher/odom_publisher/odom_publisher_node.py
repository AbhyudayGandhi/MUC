import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
import math


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Publisher for odometry messages
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Transform broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)

        # Initialize position, orientation, and time
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Wheel parameters
        self.wheel_base = 0.475  # Distance between wheels in meters
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.encoder_resolution = 273  # Ticks per wheel revolution

        # Encoder values
        self.previous_encoder_values = [0, 0, 0, 0]
        self.encoder_values = [0, 0, 0, 0]

        # IMU orientation
        self.imu_orientation = Quaternion()

        # Subscriptions for encoder values
        self.create_subscription(Int32, 'encoder_values_1', self.encoder_callback_1, 10)
        self.create_subscription(Int32, 'encoder_values_2', self.encoder_callback_2, 10)
        self.create_subscription(Int32, 'encoder_values_3', self.encoder_callback_3, 10)
        self.create_subscription(Int32, 'encoder_values_4', self.encoder_callback_4, 10)

        # Subscription for IMU data
        self.create_subscription(Imu, 'imu/orientation', self.imu_callback, 10)

        # Timer to publish odometry
        self.create_timer(0.1, self.publish_odometry)

    def encoder_callback_1(self, msg):
        self.encoder_values[0] = msg.data

    def encoder_callback_2(self, msg):
        self.encoder_values[1] = msg.data

    def encoder_callback_3(self, msg):
        self.encoder_values[2] = msg.data

    def encoder_callback_4(self, msg):
        self.encoder_values[3] = msg.data

    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt == 0:
            return

        # Compute changes in encoder values
        delta_encoder = [
            self.encoder_values[i] - self.previous_encoder_values[i]
            for i in range(4)
        ]
        self.previous_encoder_values = self.encoder_values[:]

        # Wheel velocities
        left_velocity = sum(delta_encoder[0:2]) / 2.0 * (2 * math.pi * self.wheel_radius / self.encoder_resolution) / dt
        right_velocity = sum(delta_encoder[2:4]) / 2.0 * (2 * math.pi * self.wheel_radius / self.encoder_resolution) / dt

        vx = (left_velocity + right_velocity) / 2.0
        vth = (right_velocity - left_velocity) / self.wheel_base

        # Extract yaw from IMU quaternion
        imu_quat = self.imu_orientation
        yaw = math.atan2(
            2.0 * (imu_quat.w * imu_quat.z + imu_quat.x * imu_quat.y),
            1.0 - 2.0 * (imu_quat.y**2 + imu_quat.z**2)
        )

        # Position updates
        delta_x = vx * math.cos(yaw) * dt
        delta_y = vx * math.sin(yaw) * dt

        self.x += delta_x
        self.y += delta_y
        self.th = yaw

        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = imu_quat
        self.odom_broadcaster.sendTransform(transform)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = imu_quat
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomPublisher()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

