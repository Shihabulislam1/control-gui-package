#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
from PIL import ImageTk

class VideoStreamGUI(Node):
    def __init__(self, master):
        super().__init__('tkinter_video_viewer')
        self.master = master
        self.master.title("ROS2 Video Stream")
        
        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()
        
        # Create a label to display the video
        self.video_label = ttk.Label(master)
        self.video_label.pack(padx=10, pady=10)
        
        # Subscribe to the image topic
        self.image_sub = self.create_subscription(
            Image,
            'image_raw/uncompressed',
            self.image_callback,
            10  # QoS profile depth
        )
        
        # Add a quit button
        self.quit_button = ttk.Button(
            master,
            text="Quit",
            command=self.cleanup_and_quit
        )
        self.quit_button.pack(pady=5)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert OpenCV image to PIL format
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            # Convert PIL image to PhotoImage for Tkinter
            photo = ImageTk.PhotoImage(image=pil_image)
            
            # Update the label with new image
            self.video_label.configure(image=photo)
            self.video_label.image = photo  # Keep a reference!
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def cleanup_and_quit(self):
        """Clean up and close the window"""
        self.destroy_node()
        self.master.quit()
        self.master.destroy()

    def update(self):
        """Update the GUI and process ROS callbacks"""
        rclpy.spin_once(self, timeout_sec=0)  # Process any pending callbacks
        self.master.after(10, self.update)  # Schedule the next update

def main():
    rclpy.init()
    
    root = tk.Tk()
    app = VideoStreamGUI(root)
    
    # Set up periodic GUI updates
    app.update()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
from PIL import ImageTk

class VideoStreamGUI(Node):
    def __init__(self, master):
        super().__init__('tkinter_video_viewer')
        self.master = master
        self.master.title("ROS2 Video Stream")
        
        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()
        
        # Create a label to display the video
        self.video_label = ttk.Label(master)
        self.video_label.pack(padx=10, pady=10)
        
        # Subscribe to the image topic
        self.image_sub = self.create_subscription(
            Image,
            'image_raw/uncompressed',
            self.image_callback,
            10  # QoS profile depth
        )
        
        # Add a quit button
        self.quit_button = ttk.Button(
            master,
            text="Quit",
            command=self.cleanup_and_quit
        )
        self.quit_button.pack(pady=5)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert OpenCV image to PIL format
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            # Convert PIL image to PhotoImage for Tkinter
            photo = ImageTk.PhotoImage(image=pil_image)
            
            # Update the label with new image
            self.video_label.configure(image=photo)
            self.video_label.image = photo  # Keep a reference!
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def cleanup_and_quit(self):
        """Clean up and close the window"""
        self.destroy_node()
        self.master.quit()
        self.master.destroy()

    def update(self):
        """Update the GUI and process ROS callbacks"""
        rclpy.spin_once(self, timeout_sec=0)  # Process any pending callbacks
        self.master.after(10, self.update)  # Schedule the next update

def main():
    rclpy.init()
    
    root = tk.Tk()
    app = VideoStreamGUI(root)
    
    # Set up periodic GUI updates
    app.update()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()