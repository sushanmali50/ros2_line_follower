import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # To receive velocity commands
import pigpio

# Define GPIO pins for TB6612FNG
STBY = 4  # Standby pin (must be HIGH to enable motors)

# Motor A (Left)
AIN2 = 17
AIN1 = 27
PWMA = 22

# Motor B (Right)
BIN2 = 23
BIN1 = 24
PWMB = 25

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.get_logger().info("Motor Controller Node Started")
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon!")
            exit()
        
        # Setup GPIO
        self.pi.set_mode(STBY, pigpio.OUTPUT)
        self.pi.set_mode(AIN1, pigpio.OUTPUT)
        self.pi.set_mode(AIN2, pigpio.OUTPUT)
        self.pi.set_mode(PWMA, pigpio.OUTPUT)
        self.pi.set_mode(BIN1, pigpio.OUTPUT)
        self.pi.set_mode(BIN2, pigpio.OUTPUT)
        self.pi.set_mode(PWMB, pigpio.OUTPUT)

        # Enable the motor driver
        self.pi.write(STBY, 1)

        # Subscribe to PID-controlled velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Topic where PID publishes velocity commands
            self.cmd_vel_callback,
            10
        )

    def set_motor(self, left_speed, right_speed):
        """Control the motors using speed values (-255 to 255)"""
        # Left motor control
        if left_speed > 0:
            self.pi.write(AIN1, 1)
            self.pi.write(AIN2, 0)
        elif left_speed < 0:
            self.pi.write(AIN1, 0)
            self.pi.write(AIN2, 1)
        else:
            self.pi.write(AIN1, 0)
            self.pi.write(AIN2, 0)

        # Right motor control
        if right_speed > 0:
            self.pi.write(BIN1, 1)
            self.pi.write(BIN2, 0)
        elif right_speed < 0:
            self.pi.write(BIN1, 0)
            self.pi.write(BIN2, 1)
        else:
            self.pi.write(BIN1, 0)
            self.pi.write(BIN2, 0)

        # Set PWM speeds
        self.pi.set_PWM_dutycycle(PWMA, abs(left_speed))
        self.pi.set_PWM_dutycycle(PWMB, abs(right_speed))

    def cmd_vel_callback(self, msg):
        """Callback function to process Twist messages"""
        linear = msg.linear.x  # Forward/Backward motion
        angular = msg.angular.z  # Turning motion

        # Convert velocity to motor speeds
        max_speed = 255  # Full power
        left_speed = int((linear - angular) * max_speed)
        right_speed = int((linear + angular) * max_speed)

        # Limit speeds to range -255 to 255
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))

        self.get_logger().info(f"Left: {left_speed}, Right: {right_speed}")
        self.set_motor(left_speed, right_speed)

    def cleanup(self):
        """Stops the motors and disables pigpio"""
        self.set_motor(0, 0)
        self.pi.write(STBY, 0)
        self.pi.stop()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
