#!/usr/bin/env python3

import tkinter as tk
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2


class VideoViewer(Node):
    def __init__(self):
        super().__init__('video_viewer')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            ROSImage,
            'image_raw/uncompressed',
            self.image_callback,
            10)

        self.image = None

        # Initialize Tkinter
        self.root = tk.Tk()
        self.root.title("Main Camera Video Stream")
        self.root.geometry("640x480")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Video panel
        self.panel = tk.Label(self.root)
        self.panel.pack()

        # Start GUI update and ROS spin loops
        self.root.after(10, self.spin_ros)
        self.root.after(30, self.update_gui)

    def image_callback(self, msg):
        """Callback function to handle image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convert BGR to RGB for Tkinter
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.image = cv_image
            self.get_logger().info("Image received")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def update_gui(self):
        """Update the video stream in the Tkinter label."""
        if self.image is not None:
            try:
                img = Image.fromarray(self.image)
                imgtk = ImageTk.PhotoImage(image=img)
                self.panel.imgtk = imgtk
                self.panel.config(image=imgtk)
            except Exception as e:
                self.get_logger().error(f"Error updating image in GUI: {e}")
        else:
            self.get_logger().info("No image to display")

        # Schedule the next GUI update
        self.root.after(30, self.update_gui)

    def spin_ros(self):
        """Spin the ROS node to process callbacks."""
        try:
            rclpy.spin_once(self, timeout_sec=0)
        except Exception as e:
            self.get_logger().error(f"Error spinning ROS node: {e}")

        # Schedule the next ROS spin
        self.root.after(10, self.spin_ros)

    def on_closing(self):
        """Handle closing of the Tkinter window and cleanup."""
        self.get_logger().info("Shutting down...")
        self.destroy_node()
        rclpy.shutdown()
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    viewer = VideoViewer()
    viewer.root.mainloop()


if __name__ == '__main__':
    main()