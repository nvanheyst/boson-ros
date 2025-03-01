#!/usr/bin/env python3

import cv2
import time
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class BosonCameraNode(Node):
    def __init__(self):
        super().__init__('boson_node')
        
        self.declare_parameter('raw_video', False)
        self.declare_parameter('queue_size', 10)
        
        self.capture_raw = self.get_parameter('raw_video').value
        queue_size = self.get_parameter('queue_size').value
        
        self.image_pub = self.create_publisher(Image, 'image_raw', queue_size)
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(0 + cv2.CAP_V4L2)
        
        self.capture_raw = True

        if self.capture_raw:
            self.get_logger().info("Raw (Y16) capture enabled.")
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"Y16 "))
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
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
    
    def capture_frame(self):
        capture_success, image = self.cap.read()
        if capture_success:
            if self.capture_raw:
                if image.dtype == np.uint16:
                    normalized_frame = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                    color_map_frame = cv2.applyColorMap(normalized_frame.astype(np.uint8), cv2.COLORMAP_INFERNO)
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_map_frame, encoding="bgr8"))
                else:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_map_frame, encoding="mono16"))
                
            else:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="rgb8"))
            self.frame_count += 1
            self.get_logger().debug(f"Frame {self.frame_count}")
        else:
            self.get_logger().warn("Image capture failed")
        
        self.get_logger().debug(f"FPS: {1.0/(time.time()-self.prev_capture):.2f}")
        self.prev_capture = time.time()
    
    def destroy_node(self):
        self.get_logger().info("Releasing camera...")
        self.cap.release()
        time.sleep(1) 
        super().destroy_node()
    
    def shutdown_handler(self):
        self.get_logger().info("Shutting down cleanly...")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BosonCameraNode()

    signal.signal(signal.SIGINT, node.shutdown_handler)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
