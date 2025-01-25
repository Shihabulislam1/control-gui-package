import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import re

class RelayController(Node):
    def __init__(self):
        super().__init__('relay_controller_node')

        # Define relay channel to GPIO pin mapping
        # Example for channels 1 to 8
        self.relay_channels = {
            1: {'pin': 2, 'state': 'ON'},   # GPIO 2
            2: {'pin': 3, 'state': 'ON'},   # GPIO 3
            3: {'pin': 4, 'state': 'ON'},   # GPIO 4
            4: {'pin': 17, 'state': 'ON'},  # GPIO 17
            5: {'pin': 27, 'state': 'ON'},  # GPIO 27
            6: {'pin': 22, 'state': 'ON'},  # GPIO 22
            7: {'pin': 10, 'state': 'ON'},  # GPIO 10
            8: {'pin': 9, 'state': 'ON'},   # GPIO 9
            # Add more channels as needed
        }

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        for channel_info in self.relay_channels.values():
            pin = channel_info['pin']
            GPIO.setup(pin, GPIO.OUT)
            # Initially set to high (Relay is ON)
            GPIO.output(pin, GPIO.HIGH)

        # Subscribe to the relay switch controller topic
        self.create_subscription(
            String,
            'relay_switch_controller',
            self.relay_command_callback,
            10
        )
        self.get_logger().info('Subscribed to relay_switch_controller topic')

    def relay_command_callback(self, msg):
        command_str = msg.data.strip()
        self.get_logger().info(f'Received command: {command_str}')
        # Expected command format: 'Channel X ON' or 'Channel X OFF'
        match = re.match(r'Channel\s+(\d+)\s+(ON|OFF)', command_str, re.IGNORECASE)
        if match:
            channel = int(match.group(1))
            command = match.group(2).upper()
            if channel in self.relay_channels:
                self.control_relay(channel, command)
            else:
                self.get_logger().warn(f'Unknown relay channel: {channel}')
        else:
            self.get_logger().warn(f'Invalid command format: {command_str}')

    def control_relay(self, channel, command):
        pin = self.relay_channels[channel]['pin']
        if command == 'ON':
            GPIO.output(pin, GPIO.HIGH)
            self.relay_channels[channel]['state'] = 'ON'
            self.get_logger().info(f'Relay channel {channel} turned ON')
        elif command == 'OFF':
            GPIO.output(pin, GPIO.LOW)
            self.relay_channels[channel]['state'] = 'OFF'
            self.get_logger().info(f'Relay channel {channel} turned OFF')
        else:
            self.get_logger().warn(f'Unknown command "{command}" for relay channel {channel}')

    def destroy_node(self):
        # Cleanup GPIO pins before exiting
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RelayController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()