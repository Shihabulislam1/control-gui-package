import tkinter as tk
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2

class VideoGUI:
    def __init__(self):
        # Initialize TKinter
        self.root = tk.Tk()
        self.root.title("Main Cam Video")
        # geometry of 1024x768
        self.root.geometry("1024x768")

        # Initialize ROS 2 and the Node
        rclpy.init()
        self.node = Node('video_gui_node')

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_subscriber = self.node.create_subscription(
            ROSImage,
            'image_raw/uncompressed',  # Replace with your actual topic
            self.image_callback,
            10
        )

        # Initialize the image variable and the GUI panel
        self.image = None
        self.panel = tk.Label(self.root)
        self.panel.pack()

        # Start the ROS spin and image update loops
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.update_image()
        self.spin_ros()

        # Start the TKinter main loop
        self.root.mainloop()

    def image_callback(self, msg):
        """Receive images from ROS and convert them to OpenCV format"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convert BGR to RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Store the image
            self.image = cv_image
            print("Image received")  # Debug print
        except Exception as e:
            print(f"Error converting image: {e}")

    def update_image(self):
        """Update the image in the TKinter GUI"""
        if self.image is not None:
            try:
                # Convert the OpenCV image to PIL format
                img = Image.fromarray(self.image)
                # Convert to ImageTk format
                imgtk = ImageTk.PhotoImage(image=img)
                # Update the panel with the new image
                self.panel.imgtk = imgtk
                self.panel.config(image=imgtk)
            except Exception as e:
                print(f"Error updating image in GUI: {e}")
        # Schedule the next image update
        self.root.after(30, self.update_image)  # Update at approximately 30 FPS

    def spin_ros(self):
        """Spin the ROS node to handle callbacks"""
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception as e:
            print(f"Error spinning ROS node: {e}")
        # Schedule the next ROS spin
        self.root.after(10, self.spin_ros)

    def on_close(self):
        """Handle closing of the TKinter window and cleanup"""
        print("Shutting down...")
        self.image_subscriber.destroy()
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

def main():
    gui = VideoGUI()

if __name__ == '__main__':
    main()