import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToSTM32(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_stm32')
        
        # Declare and get serial port parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Connected to STM32 on {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to STM32: {e}')
            self.serial_conn = None
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def cmd_vel_callback(self, msg):
        if self.serial_conn and self.serial_conn.is_open:
            linear_vel = msg.linear.x
            angular_vel = msg.angular.z
            send_string = f'{linear_vel} {angular_vel}\n'
            
            try:
                self.serial_conn.write(send_string.encode('utf-8'))
                self.get_logger().info(f'Sent to STM32: {send_string.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    def destroy_node(self):
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSTM32()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

