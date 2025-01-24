import tkinter as tk
from tkinter import ttk
from std_msgs.msg import String as ROSString
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2


class ControlGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        # Initialize window tracking dictionary as instance variable
        self._open_windows = {}  
        self.title("MERR Control Panel")
        self.geometry("1024x600")
        self.configure(bg="#ffffff")

         # Initialize ROS2
        try:
            rclpy.init()
        except Exception as e:
            print(f"Failed to initialize ROS2: {e}")
            self.quit()

        # Defining publishers
        self.node = Node('control_gui_node')
        self.cmd_vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_command_publisher = self.node.create_publisher(ROSString, 'arm_command', 10)
        self.actuator_command_publishers = [
            self.node.create_publisher(ROSString, f'actuator{i+1}_command', 10) for i in range(5)
        ]
        self.relay_switch_publisher = self.node.create_publisher(ROSString, 'relay_switch_controller', 10)

        self.drill_motor_publisher = self.node.create_publisher(ROSString, 'drill_motor_control', 10)

        self.drill_motor_position_motor_publisher = self.node.create_publisher(ROSString, 'drill_motor_position_motor', 10)

        self.soil_sensor_motor_publisher = self.node.create_publisher(ROSString, 'soil_sensor_motor_control', 10)

        self.science_motor1_publisher = self.node.create_publisher(ROSString, 'science_motor1_control', 10)
        self.science_motor2_publisher = self.node.create_publisher(ROSString, 'science_motor2_control', 10)


        # Main container
        self.container = tk.Frame(self)
        self.container.pack(fill="both", expand=True)

        # Sidebar
        self.sidebar = Sidebar(self.container, self)
        self.sidebar.pack(side="left", fill="y")
        # Start spinning the node
        self.spin()
        # Frame for Tabs
        # self.main_frame = tk.Frame(self.container, bg="#bdc3c7")
        # self.main_frame.pack(side="right", fill="both", expand=True)

        
    def open_tab_in_new_window(self, tab_class):
        if tab_class.__name__ in self._open_windows:
            if self._open_windows[tab_class.__name__].winfo_exists():
                self._open_windows[tab_class.__name__].focus()
                return
                
        new_window = tk.Toplevel(self)
        new_window.title(tab_class.__name__)
        new_window.geometry("800x600")
        
        tab = tab_class(new_window, 
                        self.node,
                        self.cmd_vel_publisher, 
                        self.arm_command_publisher, 
                        self.actuator_command_publishers, 
                        self.relay_switch_publisher,
                        self.drill_motor_publisher,
                        self.drill_motor_position_motor_publisher,
                        self.soil_sensor_motor_publisher,
                        self.science_motor1_publisher,
                        self.science_motor2_publisher,)

        tab.pack(fill="both", expand=True)
        self._open_windows[tab_class.__name__] = new_window
        
        def on_closing():
            if hasattr(tab, 'destroy_node'):
                tab.destroy_node()
            new_window.destroy()
            del self._open_windows[tab_class.__name__]
            
        new_window.protocol("WM_DELETE_WINDOW", on_closing)

    def spin(self):
        """Handle ROS callbacks for the main node"""
        if hasattr(self, 'node'):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.after(10, self.spin)

    def spin(self):
        """Handle ROS callbacks for the main node"""
        if hasattr(self, 'node'):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.after(10, self.spin)
    def cleanup(self):
        """Clean up ROS nodes before destroying the window"""
        print("Cleaning up ROS nodes...")
        for window in self._open_windows.values():
            if window.winfo_exists():
                window.destroy()
        rclpy.shutdown()
        self.quit()

    def __del__(self):
        # Cleanup ROS2
        rclpy.cleanup()







class Sidebar(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="#2c3e50", width=200)
        self.controller = controller  # Reference to the main app
        
        self.pack_propagate(False)
        
        # Sidebar Buttons
        self.create_button("TAB 1", Tab1)
        self.create_button("TAB 2", Tab2)
        self.create_button("TAB 3", Tab3)

    def create_button(self, text, tab_class):
        button = tk.Button(self, text=text, command=lambda: self.controller.open_tab_in_new_window(tab_class), 
                           font=("Arial", 12), bg="#34495e", fg="white", bd=0, relief="flat")
        button.pack(fill="x", pady=5, padx=10)





class Tab1(tk.Frame):
    def __init__(self, parent,node, cmd_vel_publisher, arm_command_publisher, actuator_command_publishers, relay_switch_publisher, drill_motor_publisher, drill_motor_position_motor_publisher, soil_sensor_motor_publisher, science_motor1_publisher, science_motor2_publisher):
        tk.Frame.__init__(self, parent, bg="white")
        
        self.node=node
       

    

        # Use publishers passed from ControlGUI
        self.cmd_vel_publisher = cmd_vel_publisher
        self.arm_command_publisher = arm_command_publisher
        self.actuator_command_publishers = actuator_command_publishers
        self.relay_switch_publisher = relay_switch_publisher
        self.drill_motor_publisher = drill_motor_publisher



        # Frames for side-by-side layout
        self.control_frames_container = tk.Frame(self, bg="white")
        self.control_frames_container.pack(pady=10, fill=tk.X)

        # Rover control section
        self.rover_control_section = tk.Frame(self.control_frames_container, bg="white")
        self.rover_control_section.pack(side=tk.LEFT, padx=10, pady=10)

        self.rover_label = tk.Label(self.rover_control_section, text="Rover Control", font=("Arial", 14, "bold"), bg="white")
        self.rover_label.pack()

        self.rover_control_frame = tk.Frame(self.rover_control_section, bg="white")
        self.rover_control_frame.pack(pady=5)

        self.forward_button = tk.Button(self.rover_control_frame, text="Forward", command=self.move_forward, font=("Arial", 12))
        self.forward_button.grid(row=0, column=1, padx=5, pady=5)

        self.left_button = tk.Button(self.rover_control_frame, text="Left", command=self.turn_left, font=("Arial", 12))
        self.left_button.grid(row=1, column=0, padx=5, pady=5)

        self.stop_button = tk.Button(self.rover_control_frame, text="Stop", command=self.stop_movement, font=("Arial", 12))
        self.stop_button.grid(row=1, column=1, padx=5, pady=5)

        self.right_button = tk.Button(self.rover_control_frame, text="Right", command=self.turn_right, font=("Arial", 12))
        self.right_button.grid(row=1, column=2, padx=5, pady=5)

        self.backward_button = tk.Button(self.rover_control_frame, text="Backward", command=self.move_backward, font=("Arial", 12))
        self.backward_button.grid(row=2, column=1, padx=5, pady=5)

        # Speed control slider
        self.speed_slider = tk.Scale(self.rover_control_section, from_=0, to=100, resolution=1, orient=tk.HORIZONTAL, label="Speed")
        self.speed_slider.set(0.5)  # Default speed
        self.speed_slider.pack(pady=5)

        # Actuator control section
        self.actuator_control_section = tk.Frame(self.control_frames_container, bg="white")
        self.actuator_control_section.pack(side=tk.LEFT, padx=10, pady=10)

        self.actuator_label = tk.Label(self.actuator_control_section, text="Arm Control", font=("Arial", 14, "bold"), bg="white")
        self.actuator_label.pack()

        self.actuator_control_frame = tk.Frame(self.actuator_control_section, bg="white")
        self.actuator_control_frame.pack(pady=5)

        for i in range(5):
            self.create_actuator_control_buttons(self.actuator_control_frame, i)


        # Relay switch control section
        self.relay_switch_control_section = tk.Frame(self.control_frames_container, bg="white")
        self.relay_switch_control_section.pack(side=tk.LEFT, padx=10, pady=10)

        self.relay_switch_label = tk.Label(self.relay_switch_control_section, text="Relay Switch Control", font=("Arial", 14, "bold"), bg="white")
        self.relay_switch_label.pack()

        self.relay_switch_control_frame = tk.Frame(self.relay_switch_control_section, bg="white")
        self.relay_switch_control_frame.pack(pady=5)

        for i in range(25):
            self.create_relay_switch_buttons(self.relay_switch_control_frame, i)



        # Drill motor control section
        self.drill_motor_control_section = tk.Frame(self, bg="white")
        self.drill_motor_control_section.pack(pady=10, fill=tk.X)

        label = tk.Label(self, text="Drill Motor Control", font=("Arial", 16))
        label.pack(pady=10)

        # Drill motor control buttons
        self.on_button = tk.Button(self, text="Turn Drill Motor ON", command=self.turn_drill_motor_on, font=("Arial", 12))
        self.on_button.pack(pady=5)

        self.off_button = tk.Button(self, text="Turn Drill Motor OFF", command=self.turn_drill_motor_off, font=("Arial", 12))
        self.off_button.pack(pady=5)

        print("Tab1 initialization complete")



    def create_actuator_control_buttons(self, parent, index):
        label = tk.Label(parent, text=f"Actuator {index+1}", font=("Arial", 12), bg="white")
        label.grid(row=index, column=0, padx=5, pady=5)

        start_button = tk.Button(parent, text="Start", command=lambda: self.send_actuator_command(index, "start"), font=("Arial", 12))
        start_button.grid(row=index, column=1, padx=5, pady=5)

        stop_button = tk.Button(parent, text="Stop", command=lambda: self.send_actuator_command(index, "stop"), font=("Arial", 12))
        stop_button.grid(row=index, column=2, padx=5, pady=5)

        reverse_button = tk.Button(parent, text="Reverse", command=lambda: self.send_actuator_command(index, "reverse"), font=("Arial", 12))
        reverse_button.grid(row=index, column=3, padx=5, pady=5)

    def create_relay_switch_buttons(self, parent, index):
        col = index // 5  # Determine the column (0, 1, 2, or 3)
        row = index % 5   # Determine the row (0 to 6)

        label = tk.Label(parent, text=f"Channel {index+1}", font=("Arial", 12), bg="white")
        label.grid(row=row, column=col*3, padx=5, pady=5)  # Place the label at the appropriate row and column

        on_button = tk.Button(parent, text="ON", command=lambda: self.send_relay_switch_command(index, "ON"), font=("Arial", 12))
        on_button.grid(row=row, column=col*3+1, padx=5, pady=5)  # Place the ON button next to the label

        off_button = tk.Button(parent, text="OFF", command=lambda: self.send_relay_switch_command(index, "OFF"), font=("Arial", 12))
        off_button.grid(row=row, column=col*3+2, padx=5, pady=5)  # Place the OFF button next to the ON button


    def move_forward(self):
        speed = self.speed_slider.get()/100
        self.send_twist_command(speed, 0.0)
        print("Moving forward", speed, 0.0)

    def move_backward(self):
        speed = self.speed_slider.get()/100
        self.send_twist_command(-speed, 0.0)
        print("Moving backward", -speed, 0.0)

    def turn_left(self):
        speed = self.speed_slider.get()/100
        self.send_twist_command(0.0, speed)
        print("Turning left", 0.0, speed)

    def turn_right(self):
        speed = self.speed_slider.get()/100
        self.send_twist_command(0.0, -speed)
        print("Turning right", 0.0, -speed)

    def stop_movement(self):
        self.send_twist_command(0.0, 0.0)
        print("Stopping movement", 0.0, 0.0)

    def send_twist_command(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)
        
    def start_arm(self):
        self.send_arm_command("start")
        print("Starting arm")

    def stop_arm(self):
        self.send_arm_command("stop")
        print("Stopping arm")

    def turn_off_arm(self):
        self.send_arm_command("off")
        print("Turning off arm")

    def reverse_arm(self):
        self.send_arm_command("reverse")
        print("Reversing arm")

    def send_arm_command(self, command):
        arm_command = ROSString()
        arm_command.data = command
        self.arm_command_publisher.publish(arm_command)

    def send_actuator_command(self, index, command):
        actuator_command = ROSString()
        actuator_command.data = command
        self.actuator_command_publishers[index].publish(actuator_command)
        print(f"Sending {command} command to actuator {index+1}")

    def send_relay_switch_command(self, index, command):
        relay_command = ROSString()
        relay_command.data = f"Channel {index+1} {command}"
        self.relay_switch_publisher.publish(relay_command)
        print(f"Sending {command} command to Channel {index+1}")
    
    def turn_drill_motor_on(self):
        """Send 'on' command to the drill motor."""
        message = ROSString()
        message.data = "on"
        self.drill_motor_publisher.publish(message)
        print("Drill motor ON command sent")

    def turn_drill_motor_off(self):
        """Send 'off' command to the drill motor."""
        message = ROSString()
        message.data = "off"
        self.drill_motor_publisher.publish(message)
        print("Drill motor OFF command sent")




class Tab2(tk.Frame):
    def __init__(self, parent, node,cmd_vel_publisher, arm_command_publisher, actuator_command_publishers, relay_switch_publisher, drill_motor_publisher, drill_motor_position_motor_publisher, soil_sensor_motor_publisher, science_motor1_publisher, science_motor2_publisher):
        super().__init__(parent, bg="white")

        label = tk.Label(self, text="TAB 2: 5 DC Motor Control", font=("Arial", 16))
        label.pack(pady=10)

        # Stream on/off button (Placeholder)
        self.stream_button = tk.Button(self, text="Toggle Video Stream", font=("Arial", 12))
        self.stream_button.pack(pady=5)

        # Motor control buttons
        self.start_button = tk.Button(self, text="Start Motor", font=("Arial", 12))
        self.start_button.pack(pady=5)

        self.reverse_button = tk.Button(self, text="Reverse Motor", font=("Arial", 12))
        self.reverse_button.pack(pady=5)

        self.stop_button = tk.Button(self, text="Stop Motor", font=("Arial", 12))
        self.stop_button.pack(pady=5)


class Tab3(tk.Frame):
    def __init__(self, parent, cmd_vel_publisher, arm_command_publisher, actuator_command_publishers, relay_switch_publisher, drill_motor_publisher, drill_motor_position_motor_publisher, soil_sensor_motor_publisher, science_motor1_publisher, science_motor2_publisher):
        super().__init__(parent, bg="white")

        label = tk.Label(self, text="TAB 3: Video Streams & Sensor Data", font=("Arial", 16))
        label.pack(pady=10)

        # Stream on/off button (Placeholder)
        self.stream_button = tk.Button(self, text="Toggle Video Streams", font=("Arial", 12))
        self.stream_button.pack(pady=5)

        # Placeholder for sensor data
        self.sensor_label = tk.Label(self, text="Sensor Data: N/A", font=("Arial", 12))
        self.sensor_label.pack(pady=10)












def main():
    try:
        app = ControlGUI()
        app.protocol("WM_DELETE_WINDOW", app.cleanup)  # Proper cleanup on window close
        app.mainloop()
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
