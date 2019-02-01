#!/usr/bin/python

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

def write_buffer(filename, buffer):
    t = time.time()
    np.save(filename, buffer)
    #print("Saved in {}".format(time.time()-t), filename)

def main(args):
    rospy.init_node("camera", anonymous=True)
    image_pub = rospy.Publisher("image",Image, queue_size=10)

    bridge = CvBridge()

    cap = cv2.VideoCapture(0 + cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"Y16 "))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, False)

    buffer_size = 100
    buffer = np.zeros((buffer_size,512,640), dtype='uint16')

    i = 0
    j = 0

    while not rospy.is_shutdown():
       t = time.time()
       res, image = cap.read()
       if res:
           image_pub.publish(bridge.cv2_to_imgmsg(image, encoding="passthrough"))
       #cv2.imwrite("/boson_ws/captures/image_{}.tiff".format(i), image)

           buffer[i] = image
           i += 1

       if i == buffer_size:
           #print("Writing")
           #filename = "/boson_ws/captures/buffer_{}".format(j)
           #np.save(filename, buffer)
           #buffer, buffer_double = buffer_double, buffer
           i = 0
           j += 1

       #print "FPS", 1.0/(time.time()-t)


    cap.release()

if __name__ == "__main__":
    main(sys.argv)
