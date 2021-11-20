#! /usr/bin/env python3
'''
subscribe to realsense RGB image
read densepose infer out
publish IUV
'''
import os
import rospy
import numpy as np
from cv2 import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

image_input = None
IUV = None


def image_callback(msg):
  global image_input
  image_input = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")


rospy.init_node('IUVPublisher', anonymous=True)
IUV_pub = rospy.Publisher('IUV', Image, queue_size=10)
image_sub = rospy.Subscriber('DensePoseInput', Image, image_callback)

incoming_path = os.path.join(os.path.expanduser('~'), 'DensePose/DensePoseData/image_buffer/incoming.png')
IUV_path = os.path.join(os.path.expanduser('~'), 'DensePose/DensePoseData/infer_out/incoming_IUV.png')

while not rospy.is_shutdown():
  if image_input is not None:
    cv2.imwrite(incoming_path, image_input)
  try:
    IUV = cv2.imread(IUV_path)
  except:
    print('IUV not available')
  if IUV is not None:
    IUV_pub.publish(CvBridge().cv2_to_imgmsg(IUV, encoding="passthrough"))
  else:
    print("no target detected")
