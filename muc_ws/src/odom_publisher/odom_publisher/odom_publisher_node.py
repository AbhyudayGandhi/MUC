import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu  # Import the IMU message type
import math

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Create a publisher for odometry messages
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create a TransformBroadcaster to publish transforms
        self.odom_broadcaster = TransformBroadcaster(self)

        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Initialize time
        self.last_time = self.get_clock().now()

        # Wheel parameters
        self.wheel_base = 0.475  # Distance between left and right wheels in meters
        self.wheel_radius = 0.05  # Radius of the wheels in meters
        self.encoder_resolution = 273  # Ticks per wheel revolution

        # Encoder values
        self.previous_encoder_values = [0, 0, 0, 0]  # To store previous encoder ticks
        self.encoder_values = [0, 0, 0, 0]  # For four motors

        # IMU quaternion values (initialize to 0, as we'll update from IMU)
        self.imu_orientation = Quaternion()

        # Subscribe to encoder topics
        self.subscription_1 = self.create_subscription(
            Int32,
            'encoder_values_1',
            self.encoder_callback_1,
            10
        )
        self.subscription_2 = self.create_subscription(
            Int32,
            'encoder_values_2',
            self.encoder_callback_2,
            10
        )
        self.subscription_3 = self.create_subscription(
            Int32,
            'encoder_values_3',
            self.encoder_callback_3,
            10
        )
        self.subscription_4 = self.create_subscription(
            Int32,
            'encoder_values_4',
            self.encoder_callback_4,
            10
        )

        # Subscribe to IMU topic (imu_publisher)
        self.imu_subscription = self.create_subscription(
            Imu,                      # Message type (IMU data)
            'imu/orientation',           # Topic name
            self.imu_callback,         # Callback function to handle IMU data
            10                         # QoS (Quality of Service) Depth
        )

        # Timer to publish odometry
        self.create_timer(0.1, self.publish_odometry)  # Adjust the timer interval as needed

    def encoder_callback_1(self, msg):
        self.encoder_values[0] = msg.data
        
    def encoder_callback_2(self, msg):
        self.encoder_values[1] = msg.data
        
    def encoder_callback_3(self, msg):
        self.encoder_values[2] = msg.data
        
    def encoder_callback_4(self, msg):
        self.encoder_values[3] = msg.data

    def imu_callback(self, msg):
        # Callback function for IMU data
        # Extract and store the IMU orientation
        self.imu_orientation = msg.orientation
        self.get_logger().info(f"Received IMU data: Orientation - {self.imu_orientation}")

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt == 0:
            return

        # Calculate changes in encoder values
        delta_encoder = [
            self.encoder_values[i] - self.previous_encoder_values[i]
            for i in range(4)
        ]

        # Update previous encoder values
        self.previous_encoder_values = self.encoder_values[:]

        # Calculate wheel velocities (ticks/s to m/s)
        left_velocity = sum(delta_encoder[0:2]) / 2.0 * (2 * math.pi * self.wheel_radius / self.encoder_resolution) / dt
        right_velocity = sum(delta_encoder[2:4]) / 2.0 * (2 * math.pi * self.wheel_radius / self.encoder_resolution) / dt

        vx = (left_velocity + right_velocity) / 2.0
        vth = (right_velocity - left_velocity) / self.wheel_base

        # Log velocities
        self.get_logger().info(f"Linear Velocity: {vx:.2f} m/s, Angular Velocity: {vth:.2f} rad/s")

        # Calculate changes in position and orientation
        delta_x = vx * math.cos(self.th) * dt
        delta_y = vx * math.sin(self.th) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Use the IMU orientation for odometry quaternion
        odom_quat = self.imu_orientation  # Use the IMU quaternion for the odometry

        # Publish the transform over tf
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_quat

        self.odom_broadcaster.sendTransform(transform)

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # Update last time
        self.last_time = current_time

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomPublisher()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
