import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String  # Use std_msgs/String for text commands

MOTOR_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

class DrillMotorController(Node):
    def __init__(self):
        super().__init__('drill_motor_controller')
        self.subscription = self.create_subscription(
            String,
            'drill_motor_control',
            self.motor_command_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Assume motor is initially OFF
        self.motor_state = False

    def motor_command_callback(self, msg):
        """ Callback function to handle incoming commands. """
        command = msg.data.lower().strip()

        if command == "on":
            self.turn_motor_on()
        elif command == "off":
            self.turn_motor_off()
        else:
            self.get_logger().warn(f"Received unknown command: {command}")

    def turn_motor_on(self):
        if not self.motor_state:
            GPIO.output(MOTOR_PIN, GPIO.HIGH)
            self.motor_state = True
            self.get_logger().info("Drill motor is now ON")
            # Code to turn the actual motor on (e.g., GPIO control)
        else:
            self.get_logger().info("Drill motor is already ON")

    def turn_motor_off(self):
        if self.motor_state:
            GPIO.output(MOTOR_PIN, GPIO.LOW)
            self.motor_state = False
            self.get_logger().info("Drill motor is now OFF")
            # Code to turn the actual motor off (e.g., GPIO control)
        else:
            self.get_logger().info("Drill motor is already OFF")

def main(args=None):
    rclpy.init(args=args)
    node = DrillMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
