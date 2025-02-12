#!/usr/bin/env python3

import roslib
roslib.load_manifest("boson")
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def main(args):
    
    rospy.init_node("camera", anonymous=True)
   
    # Need to use the V4L2 backend on Linux to get the right format
    cap = cv2.VideoCapture(0 + cv2.CAP_V4L2)
    
    # Set fourcc code to Y16 and disable RGB conversion
    capture_raw = rospy.get_param('~raw_video', default=False)

    if capture_raw:
        rospy.loginfo("Raw (Y16) capture enabled.")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"Y16 "))
        cap.set(cv2.CAP_PROP_CONVERT_RGB, False)
    else:
        rospy.loginfo("RGB24 capture enabled.")

    # Set default resolution (Boson 640)
    default_width = 640
    default_height = 512
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, default_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, default_height)
    
    # Get actual resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    if width < default_width and height < default_height:
        rospy.loginfo(f"Detected lower resolution {width}x{height}, adjusting...")
    else:
        rospy.loginfo(f"Using resolution {width}x{height}")

    queue_size = rospy.get_param('~queue_size', default=10)
    image_pub = rospy.Publisher("image_raw", Image, queue_size=queue_size)

    bridge = CvBridge()
    frame_count = 0
    prev_capture = time.time()

    while not rospy.is_shutdown():

        capture_success, image = cap.read()

        if capture_success:
            if capture_raw:
                image_pub.publish(bridge.cv2_to_imgmsg(image, encoding="mono16"))
            else:
                image_pub.publish(bridge.cv2_to_imgmsg(image, encoding="rgb8"))
            frame_count += 1
            rospy.logdebug("Frame %d", frame_count)
        else:
            rospy.logwarn(5, "Image capture failed")

        rospy.logdebug("FPS: %f", 1.0/(time.time()-prev_capture))
        prev_capture = time.time()

    cap.release()

if __name__ == "__main__":
    main(sys.argv)
