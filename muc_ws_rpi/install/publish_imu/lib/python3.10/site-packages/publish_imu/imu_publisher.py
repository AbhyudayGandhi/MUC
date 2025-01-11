import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

class OrientationPublisher(Node):
    def __init__(self):
        super().__init__('orientation_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/orientation', 10)
        timer_period = 0.1  # Publish at 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_orientation)

        # Initialize IMU
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.get_logger().info('BNO08X IMU Initialized and Rotation Vector Feature Enabled')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO08X IMU: {e}')
            rclpy.shutdown()

    def publish_orientation(self):
        msg = Imu()

        try:
            # Retrieve orientation data (quaternion)
            quat_x, quat_y, quat_z, quat_w = self.bno.quaternion

            # Fill the orientation field of the IMU message
            msg.orientation.x = quat_x
            msg.orientation.y = quat_y
            msg.orientation.z = quat_z
            msg.orientation.w = quat_w

            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'Published Orientation: x={quat_x:.2f}, y={quat_y:.2f}, z={quat_z:.2f}, w={quat_w:.2f}'
            )

        except KeyError as e:
            # Handle unknown report types or invalid data
            self.get_logger().error(f'KeyError: {e} - Possible unsupported report type or sensor misconfiguration.')
        except Exception as e:
            # Catch other errors related to sensor communication
            self.get_logger().error(f'Error retrieving quaternion data: {e}')

def main(args=None):
    rclpy.init(args=args)
    orientation_publisher = OrientationPublisher()

    try:
        rclpy.spin(orientation_publisher)
    except KeyboardInterrupt:
        orientation_publisher.get_logger().info('Node stopped by user')
    except Exception as e:
        orientation_publisher.get_logger().error(f'Error in node execution: {e}')
    finally:
        orientation_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
