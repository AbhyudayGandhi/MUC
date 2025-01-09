#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

class MotorController:
    def __init__(self, pwm_pin, in_pin):
        self.pwm_pin = pwm_pin
        self.in_pin = in_pin

        # Setup GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.in_pin, GPIO.OUT)

        # Create PWM instance with 100Hz frequency
        self.pwm = GPIO.PWM(self.pwm_pin, 100)
        self.pwm.start(0)  # Start with 0% duty cycle

    def set_speed(self, speed):
        # Set the PWM duty cycle (0 to 100)
        self.pwm.ChangeDutyCycle(speed)

    def forward(self):
        # Set the motor direction to forward
        GPIO.output(self.in_pin, GPIO.HIGH)

    def backward(self):
        # Set the motor direction to backward
        GPIO.output(self.in_pin, GPIO.LOW)

    def stop(self):
        # Stop the motor
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        # Clean up PWM and GPIO
        self.pwm.stop()

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Initialize Motor Controllers with the specified pins
        self.motor1 = MotorController(pwm_pin=17, in_pin=18)
        self.motor2 = MotorController(pwm_pin=13, in_pin=6)
        self.motor3 = MotorController(pwm_pin=23, in_pin=24)
        self.motor4 = MotorController(pwm_pin=26, in_pin=22)

        # Subscribe to the PWM value topics
        self.subscription1 = self.create_subscription(Int32, 'pwm_values_1', self.pwm_callback1, 10)
        self.subscription2 = self.create_subscription(Int32, 'pwm_values_2', self.pwm_callback2, 10)
        self.subscription3 = self.create_subscription(Int32, 'pwm_values_3', self.pwm_callback3, 10)
        self.subscription4 = self.create_subscription(Int32, 'pwm_values_4', self.pwm_callback4, 10)

    def pwm_callback1(self, msg):
        pwm_value = msg.data
        self.get_logger().info(f'Received PWM Value for Motor 1: {pwm_value}')
        self._control_motor(self.motor1, pwm_value)

    def pwm_callback2(self, msg):
        pwm_value = msg.data
        self.get_logger().info(f'Received PWM Value for Motor 2: {pwm_value}')
        self._control_motor(self.motor2, pwm_value)

    def pwm_callback3(self, msg):
        pwm_value = msg.data
        self.get_logger().info(f'Received PWM Value for Motor 3: {pwm_value}')
        self._control_motor(self.motor3, pwm_value)

    def pwm_callback4(self, msg):
        pwm_value = msg.data
        self.get_logger().info(f'Received PWM Value for Motor 4: {pwm_value}')
        self._control_motor(self.motor4, pwm_value)

    def _control_motor(self, motor, pwm_value):
        if pwm_value > 0:
            motor.forward()
        elif pwm_value < 0:
            motor.backward()
        else:
            motor.stop()
        motor.set_speed(abs(pwm_value))

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        motor_control_node.get_logger().info('Keyboard Interrupt (Ctrl+C) received. Cleaning up.')
    finally:
        # Cleanup all motors and GPIO
        motor_control_node.motor1.cleanup()
        motor_control_node.motor2.cleanup()
        motor_control_node.motor3.cleanup()
        motor_control_node.motor4.cleanup()
        GPIO.cleanup()
        motor_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


