import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class ActuatorController(Node):
    def __init__(self):
        super().__init__('actuator_controller_node')

        # Define actuator pin mappings
        self.actuators = {
            0: {'name': 'Base', 'pins': (17, 27), 'state': 'stop'},
            1: {'name': 'Shoulder', 'pins': (22, 23), 'state': 'stop'},
            2: {'name': 'Elbow', 'pins': (24, 25), 'state': 'stop'},
            3: {'name': 'Wrist', 'pins': (5, 6), 'state': 'stop'},
            4: {'name': 'Gripper', 'pins': (13, 19), 'state': 'stop'},
            # Add more actuators as needed
        }

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        for actuator_info in self.actuators.values():
            pin1, pin2 = actuator_info['pins']
            GPIO.setup(pin1, GPIO.OUT)
            GPIO.setup(pin2, GPIO.OUT)
            # Set both pins high initially (stop state)
            GPIO.output(pin1, GPIO.HIGH)
            GPIO.output(pin2, GPIO.HIGH)

        # Create subscribers for each actuator
        for index, actuator_info in self.actuators.items():
            topic_name = f'actuator{index+1}_command'
            self.create_subscription(
                String,
                topic_name,
                lambda msg, idx=index: self.actuator_command_callback(msg, idx),
                10
            )
            self.get_logger().info(f'Subscribed to {topic_name} topic')

    def actuator_command_callback(self, msg, index):
        command = msg.data.lower().strip()
        actuator_info = self.actuators[index]
        actuator_name = actuator_info['name']
        self.get_logger().info(f'Received command for {actuator_name}: {command}')
        if command == 'start':
            self.start_actuator(index)
        elif command == 'stop':
            self.stop_actuator(index)
        elif command == 'reverse':
            self.reverse_actuator(index)
        else:
            self.get_logger().warn(f'Unknown command "{command}" for {actuator_name}')

    def start_actuator(self, index):
        pin1, pin2 = self.actuators[index]['pins']
        GPIO.output(pin1, GPIO.HIGH)
        GPIO.output(pin2, GPIO.LOW)
        self.actuators[index]['state'] = 'start'
        actuator_name = self.actuators[index]['name']
        self.get_logger().info(f'{actuator_name} started (Forward)')

    def stop_actuator(self, index):
        pin1, pin2 = self.actuators[index]['pins']
        GPIO.output(pin1, GPIO.HIGH)
        GPIO.output(pin2, GPIO.HIGH)
        self.actuators[index]['state'] = 'stop'
        actuator_name = self.actuators[index]['name']
        self.get_logger().info(f'{actuator_name} stopped')

    def reverse_actuator(self, index):
        pin1, pin2 = self.actuators[index]['pins']
        GPIO.output(pin1, GPIO.LOW)
        GPIO.output(pin2, GPIO.HIGH)
        self.actuators[index]['state'] = 'reverse'
        actuator_name = self.actuators[index]['name']
        self.get_logger().info(f'{actuator_name} started (Reverse)')

    def destroy_node(self):
        # Cleanup GPIO pins before exiting
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()