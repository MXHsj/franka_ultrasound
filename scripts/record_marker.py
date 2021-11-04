#! /usr/bin/env python3
'''
record aruco marker positions in realsense RGB image
'''
import csv
import numpy as np
from cv2 import cv2
from cv2 import aruco
from GetRealSenseData import GetRealSenseData


class RecordMarker:
  def __init__(self):
    num_markers = 8


def detectMarker(frame):
  # marker detection
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
  parameters = aruco.DetectorParameters_create()
  corners, ids, _ = aruco.detectMarkers(
      gray, aruco_dict, parameters=parameters)
  marker_frame = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

  return marker_frame, corners, ids


def trackMarker(corners, ids):
  num_markers = 8
  pos = np.zeros((num_markers, 2))
  for i in range(num_markers):
    try:
      marker = corners[np.where(ids == i)[0][0]][0]
      pos[i, :] = [marker[:, 0].mean(), marker[:, 1].mean()]
    except:
      pos[i, :] = [-1, -1]      # if marker is not detected
    # print("id{} center:".format(i), pos[i-1, 0], pos[i-1, 1])
  return pos


def main():
  rs_data = GetRealSenseData()
  cv2.namedWindow('realsense', cv2.WINDOW_AUTOSIZE)
  while True:
    rs_data.stream_color_frame()
    key = cv2.waitKey(10)
    mk_frame, corners, ids = detectMarker(rs_data.color_image)
    pos = trackMarker(corners, ids)
    cv2.imshow('realsense', mk_frame)
    if key & 0xFF == ord('q') or key == 27:
      rs_data.stop_stream()
      break


if __name__ == "__main__":
  main()
