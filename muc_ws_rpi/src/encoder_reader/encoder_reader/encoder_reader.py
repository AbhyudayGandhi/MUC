import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_publisher')

        # ROS2 Publishers for each motor
        self.motor1_pub = self.create_publisher(Int32, 'encoder_motor1', 10)
        self.motor2_pub = self.create_publisher(Int32, 'encoder_motor2', 10)
        self.motor3_pub = self.create_publisher(Int32, 'encoder_motor3', 10)
        self.motor4_pub = self.create_publisher(Int32, 'encoder_motor4', 10)

        # Open Serial Port (Modify if needed)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)  
            self.serial_port.reset_input_buffer()  # Flush buffer on startup
            self.get_logger().info("Serial connection established on /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # Faster polling rate (20Hz instead of 10Hz)
        self.timer = self.create_timer(0.01, self.read_encoder_data)  

    def read_encoder_data(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith("M1:"):
                    parts = line.replace("M1: ", "").replace("| M2: ", "").replace("| M3: ", "").replace("| M4: ", "").split()

                    if len(parts) == 4:
                        motor_ticks = [int(p) for p in parts]

                        # Publish encoder readings
                        self.motor1_pub.publish(Int32(data=motor_ticks[0]))
                        self.motor2_pub.publish(Int32(data=motor_ticks[1]))
                        self.motor3_pub.publish(Int32(data=motor_ticks[2]))
                        self.motor4_pub.publish(Int32(data=motor_ticks[3]))

                        # Log encoder readings
                        self.get_logger().info(f"ENCODER READINGS -> M1: {motor_ticks[0]} | M2: {motor_ticks[1]} | M3: {motor_ticks[2]} | M4: {motor_ticks[3]}")

        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






