#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import time
import signal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class BosonCameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        
        self.declare_parameter('raw_video', False)
        self.declare_parameter('queue_size', 10)
        
        self.capture_raw = self.get_parameter('raw_video').value
        queue_size = self.get_parameter('queue_size').value
        
        self.image_pub = self.create_publisher(Image, 'image_raw', queue_size)
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(0 + cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera. Exiting...")
            rclpy.shutdown()
            return
        
        if self.capture_raw:
            self.get_logger().info("Raw (Y16) capture enabled.")
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"Y16 "))
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, False)
        else:
            self.get_logger().info("RGB24 capture enabled.")
        
        self.default_width = 640
        self.default_height = 512
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.default_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.default_height)
        
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        if self.width < self.default_width and self.height < self.default_height:
            self.get_logger().info(f"Detected lower resolution {self.width}x{self.height}, adjusting...")
        else:
            self.get_logger().info(f"Using resolution {self.width}x{self.height}")
        
        self.timer = self.create_timer(0.1, self.capture_frame)
        self.frame_count = 0
        self.prev_capture = time.time()
        
        # Handle SIGINT (Ctrl+C) to ensure proper cleanup
        signal.signal(signal.SIGINT, self.shutdown_handler)
    
    def capture_frame(self):
        capture_success, image = self.cap.read()
        if capture_success:
            if self.capture_raw:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="mono16"))
            else:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="rgb8"))
            self.frame_count += 1
            self.get_logger().debug(f"Frame {self.frame_count}")
        else:
            self.get_logger().warn("Image capture failed")
        
        self.get_logger().debug(f"FPS: {1.0/(time.time()-self.prev_capture):.2f}")
        self.prev_capture = time.time()
    
    def shutdown_handler(self, signum, frame):
        self.get_logger().info("Shutdown signal received. Releasing camera...")
        self.destroy_node()
        rclpy.shutdown()
    
    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released successfully.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BosonCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
