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

        # Main container
        self.container = tk.Frame(self)
        self.container.pack(fill="both", expand=True)

        # Sidebar
        self.sidebar = Sidebar(self.container, self)
        self.sidebar.pack(side="left", fill="y")

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
        
        tab = tab_class(new_window)
        tab.pack(fill="both", expand=True)
        self._open_windows[tab_class.__name__] = new_window
        
        def on_closing():
            if hasattr(tab, 'destroy_node'):
                tab.destroy_node()
            new_window.destroy()
            del self._open_windows[tab_class.__name__]
            
        new_window.protocol("WM_DELETE_WINDOW", on_closing)


    def __del__(self):
        # Cleanup ROS2
        rclpy.shutdown()
















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





class Tab1(tk.Frame, Node):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent, bg="white")
        Node.__init__(self, 'tab1_node')
        
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            ROSImage,
            'ImagePublisher',
            self.image_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_command_publisher = self.create_publisher(ROSString, 'arm_command', 10)
        self.actuator_command_publishers = [
        self.create_publisher(ROSString, f'actuator{i+1}_command', 10) for i in range(5)
        ]

        # Video stream label
        self.video_label = tk.Label(self, bg="black")
        self.video_label.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)

        # Stop stream button
        self.stream_button = tk.Button(self, text="Stop Stream", command=self.stop_stream, font=("Arial", 12))
        self.stream_button.pack(pady=5)

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

        # Actuator control section
        self.actuator_control_section = tk.Frame(self.control_frames_container, bg="white")
        self.actuator_control_section.pack(side=tk.LEFT, padx=10, pady=10)

        self.actuator_label = tk.Label(self.actuator_control_section, text="Arm Control", font=("Arial", 14, "bold"), bg="white")
        self.actuator_label.pack()

        self.actuator_control_frame = tk.Frame(self.actuator_control_section, bg="white")
        self.actuator_control_frame.pack(pady=5)

        for i in range(5):
            self.create_actuator_control_buttons(self.actuator_control_frame, i)


        self.image = None
        self.update_image()


    def create_actuator_control_buttons(self, parent, index):
        label = tk.Label(parent, text=f"Actuator {index+1}", font=("Arial", 12), bg="white")
        label.grid(row=index, column=0, padx=5, pady=5)

        start_button = tk.Button(parent, text="Start", command=lambda: self.send_actuator_command(index, "start"), font=("Arial", 12))
        start_button.grid(row=index, column=1, padx=5, pady=5)

        stop_button = tk.Button(parent, text="Stop", command=lambda: self.send_actuator_command(index, "stop"), font=("Arial", 12))
        stop_button.grid(row=index, column=2, padx=5, pady=5)

        reverse_button = tk.Button(parent, text="Reverse", command=lambda: self.send_actuator_command(index, "reverse"), font=("Arial", 12))
        reverse_button.grid(row=index, column=3, padx=5, pady=5)


    def image_callback(self, msg):
        """Callback function to handle image messages."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    def update_image(self):
        """Update the video stream in the Tkinter label."""
        if self.image is not None:
            img = Image.fromarray(self.image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
        self.after(10, self.update_image)

    def stop_stream(self):
        """Stop the video stream."""
        self.image_subscriber.destroy()

    def move_forward(self):
        self.send_twist_command(0.5, 0.0)
        print("Moving forward", 0.5, 0.0)

    def move_backward(self):
        self.send_twist_command(-0.5, 0.0)
        print("Moving backward", -0.5, 0.0)

    def turn_left(self):
        self.send_twist_command(0.0, 0.5)
        print("Turning left", 0.0, 0.5)

    def turn_right(self):
        self.send_twist_command(0.0, -0.5)
        print("Turning right", 0.0, -0.5)

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



class Tab2(tk.Frame):
    def __init__(self, parent):
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
    def __init__(self, parent):
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
    app = ControlGUI()
    app.mainloop()


if __name__ == "__main__":
    main()