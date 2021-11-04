#! /usr/bin/env python3
'''
record aruco marker positions in realsense RGB image
'''
import os
import csv
import numpy as np
from cv2 import cv2
from cv2 import aruco
from GetRealSenseData import GetRealSenseData


class RecordMarker:
  __aruco_dict_4x4 = aruco.Dictionary_get(aruco.DICT_4X4_250)
  __aruco_dict_6x6 = aruco.Dictionary_get(aruco.DICT_6X6_250)
  __detect_param = aruco.DetectorParameters_create()
  save_path = os.path.join(os.path.dirname(__file__), '../data/marker_pos.csv')

  def __init__(self, num_mk=8):
    self.num_markers = num_mk
    self.mk_bbox_frame = None
    self.mk_axis_frame = None
    self.corners = None
    self.ids = None
    self.mk_pos = np.zeros((self.num_markers, 2))

  def detect_markers(self, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    self.corners, self.ids, _ = \
        aruco.detectMarkers(gray, self.__aruco_dict_6x6, parameters=self.__detect_param)
    self.mk_bbox_frame = aruco.drawDetectedMarkers(frame.copy(), self.corners, self.ids)
    for id in range(self.num_markers):
      try:
        curr_mk = corners[np.where(self.ids == id)[0][0]][0]
        self.mk_pos[id, :] = [curr_mk[:, 0].mean(), curr_mk[:, 1].mean()]
      except:
        self.mk_pos[id, :] = [-1, -1]  # if marker is not detected

  def save_mk(self):
    with open(self.save_path, 'a') as file_out:
      writer = csv.writer(file_out)
      writer.writerows(self.mk_pos)

'''
test case
'''
def test():
  rs_obj = GetRealSenseData()
  mk_obj = RecordMarker()

  cv2.namedWindow('realsense', cv2.WINDOW_AUTOSIZE)
  while True:
    rs_obj.stream_color_frame()
    mk_obj.detect_markers(rs_obj.color_image)
    mk_obj.save_mk()
    key = cv2.waitKey(10)
    cv2.imshow('realsense', mk_obj.mk_bbox_frame)
    if key & 0xFF == ord('q') or key == 27:
      rs_obj.stop_stream()
      break


if __name__ == "__main__":
  test()
