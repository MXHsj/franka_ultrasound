#! /usr/bin/env python3
'''
read pre-recorded Aruco marker position ->
read image from realsense ->
send to DensePose through ROS topic->
receive IUV through ROS topic ->
record DensePose inferred pixel targets given u, v
'''
import os
import csv
import rospy
import numpy as np
from cv2 import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from GetRealSenseData import GetRealSenseData


U_chest_tar = np.array([255/2-38, 255/2-38, 255/2-78, 255/2-78,               # anterior
                        255/2-20, 255/2-20, 255/2-65, 255/2-65], dtype=int)   # lateral
V_chest_tar = np.array([255/2-30, 255/2+30, 255/2-30, 255/2+30,               # anterior
                        255/2-80, 255/2+80, 255/2-80, 255/2+80], dtype=int)   # lateral
# U_chest_tar = np.array([60, 100, 60, 100])
# V_chest_tar = np.array([155, 155, 105, 105])
iuv_frm_path = os.path.join(os.path.dirname(__file__), '../data/densepose/IUV.png')
dp_tar_path = os.path.join(os.path.dirname(__file__), '../data/densepose/dp_target_log.csv')
color_frm_path = os.path.join(os.path.dirname(__file__), '../data/densepose/color_frame.png')
depth_frm_path = os.path.join(os.path.dirname(__file__), '../data/densepose/depth_frame.png')
IUV = None


def iuv_callback(msg):
  global IUV
  # cvbridge has error with python3, using numpy instead
  IUV = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)


def selected_uv2pix_in_image(IUV, U, V):
  '''
  calculate pixel position in RGB image given selected IUV & uv
  '''
  x_ = np.zeros(U.shape, dtype=int)
  y_ = np.zeros(U.shape, dtype=int)
  for i, (uu, vv) in enumerate(zip(U, V)):
    u2xy = np.where(IUV[:, :, 1] == uu)
    v2xy = np.where(IUV[:, :, 2] == vv)
    x_intersects = [x for x in u2xy[1] if x in v2xy[1]]
    y_intersects = [y for y in u2xy[0] if y in v2xy[0]]
    if len(x_intersects) <= 0 or len(y_intersects) <= 0:
      x_[i] = -1
      y_[i] = -1
    else:
      x_[i] = np.mean(x_intersects)
      y_[i] = np.mean(y_intersects)
    print("iter:", i, "row:", x_[i], "col:", y_[i])
  return x_, y_


def save_tar(row, col, pnts, writer):
  row2write = list()
  # append targets in pixels
  for tar in range(len(row)):
    row2write.append((row[tar], col[tar]))
  # append targets in xyz
  for tar in range(len(pnts)):
    row2write.append((pnts[tar, 0], pnts[tar, 1], pnts[tar, 2]))
  writer.writerow(row2write)


rs_obj = GetRealSenseData()
rospy.init_node('DensePoseErrRecorder', anonymous=True)
im_pub = rospy.Publisher('DensePoseInput', Image, queue_size=10)
rospy.Subscriber('IUV', Image, iuv_callback)
file_out = open(dp_tar_path, 'w')
writer = csv.writer(file_out)

cv2.namedWindow('realsense', cv2.WINDOW_AUTOSIZE)
while not rospy.is_shutdown():
  depth_frame, _ = rs_obj.stream_depth2color_aligned()
  color_padded = cv2.copyMakeBorder(rs_obj.color_image, 80, 80, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
  depth_padded = cv2.copyMakeBorder(rs_obj.depth_image, 80, 80, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
  im_pub.publish(CvBridge().cv2_to_imgmsg(color_padded, encoding="passthrough"))
  if IUV is not None:
    # get chest area
    IUV_chest = np.zeros((IUV.shape))
    chest_idx = np.where(IUV[:, :, 0] == 2)
    IUV_chest[chest_idx] = IUV[chest_idx]
    row, col = selected_uv2pix_in_image(IUV_chest, U_chest_tar, V_chest_tar)
    # get target xyz from pixels
    tar_pix = np.zeros((len(row), 2), dtype=int)
    for i in range(len(row)):
      tar_pix[i, :] = [row[i]-80, col[i]]   # row needs substract padded height
    pnts = rs_obj.get_xyz(depth_frame, tar_pix)
    save_tar(row, col, pnts, writer)
    # vis
    # for i in range(len(row)):
    #   cv2.circle(color_padded, (row[i], col[i]), 5, (200, 200, 200), thickness=1)
    img2show = np.hstack((color_padded, IUV))
  else:
    img2show = color_padded
  cv2.imshow('realsense', img2show)
  key = cv2.waitKey(10)
  if key & 0xFF == ord('q') or key == 27:
    cv2.destroyAllWindows()
    break
  elif key == ord('s'):
    cv2.imwrite(iuv_frm_path, IUV)
    cv2.imwrite(color_frm_path, color_padded)
    cv2.imwrite(depth_frm_path, depth_padded)
    print('frame saved')

file_out.close()
