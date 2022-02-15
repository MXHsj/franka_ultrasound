#! /usr/bin/env python3
'''
record aruco marker positions in realsense RGB image
'''
import os
import csv
import time
import numpy as np
from cv2 import cv2
from cv2 import aruco
from GetRealSenseData import GetRealSenseData


class TrackMarker:
  __detect_param = aruco.DetectorParameters_create()
  save_path = os.path.join(os.path.dirname(__file__), '../data/densepose/marker_pos_log.csv')

  def __init__(self, num_mk=8, dict_id=6):
    if dict_id == 6:
      self.__aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    elif dict_id == 4:
      self.__aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    self.num_markers = num_mk
    self.mk_bbox_frame = None
    self.mk_axis_frame = None
    self.corners = None
    self.ids = None
    self.mk_pos = np.zeros((self.num_markers, 2), dtype=int)
    self.mk_rmat = None
    self.mk_tvec = None
    self.file_out = open(self.save_path, 'a')
    self.writer = csv.writer(self.file_out)

  def __del__(self):
    self.file_out.close()

  def detect_markers(self, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    self.corners, self.ids, _ = aruco.detectMarkers(
        gray, self.__aruco_dict, parameters=self.__detect_param)
    self.mk_bbox_frame = aruco.drawDetectedMarkers(frame.copy(), self.corners, self.ids)
    for id in range(self.num_markers):
      try:
        curr_mk = self.corners[np.where(self.ids == id+1)[0][0]][0]
        self.mk_pos[id, :] = [round(curr_mk[:, 0].mean()), round(curr_mk[:, 1].mean())]
      except:
        self.mk_pos[id, :] = [-1, -1]  # if marker is not detected
      print('id:', id+1, ' pos', self.mk_pos[id, :])

  def estimate_pose(self, frame, cam_intri, dist_coeff, mk_id=0):
    # TODO: test this method
    try:
      loc_marker = self.corners[np.where(ids == mk_id)[0][0]]
      rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(loc_marker, 0.047, cam_intri, dist_coeff)
      self.mk_axis_frame = aruco.drawAxis(frame.copy(), camera_matrix, dist_coeff, rvecs, tvecs, 0.1)
      self.rmat = cv2.Rodrigues(rvecs)[0]      # 3x3 rotation
      self.tvec = np.transpose(tvecs)[:, 0, 0]  # 3x1 translation
      UV = [loc_marker[0][:, 0].mean(), loc_marker[0][:, 1].mean()]
    except:
      self.rmat = -1*np.ones([3, 3])
      self.tvec = [-1, -1, -1]
      UV = [-1, -1]
    # return UV, rmat[:, 2], tvec
    return UV

  def save_mk_pos(self):
    row2write = list()
    for id in range(self.num_markers):
      row2write.append((self.mk_pos[id, 0], self.mk_pos[id, 1]))
    self.writer.writerow(row2write)

  def save_mk_pos_xyz(self, pnts):
    row2write = list()
    # append markers in pixels
    for id in range(self.num_markers):
      row2write.append((self.mk_pos[id, 0], self.mk_pos[id, 1]))
    # append targets in xyz
    for id in range(self.num_markers):
      row2write.append((pnts[id, 0], pnts[id, 1], pnts[id, 2]))
    self.writer.writerow(row2write)


def main():
  frm_save_path = '../data/densepose/mk_frame.png'
  rs_obj = GetRealSenseData()
  mk_obj = TrackMarker(num_mk=4)
  cv2.namedWindow('realsense', cv2.WINDOW_AUTOSIZE)
  nFrames = 20
  freq = 0.8
  frmCount = 1
  while True:
    now = time.time()
    depth_frame, _ = rs_obj.stream_depth2color_aligned()
    color_padded = cv2.copyMakeBorder(rs_obj.color_image, 80, 80, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    mk_obj.detect_markers(color_padded)
    # get target xyz from pixels
    tar_pix = np.zeros((mk_obj.num_markers, 2), dtype=int)
    for i in range(mk_obj.num_markers):
      tar_pix[i, :] = [mk_obj.mk_pos[i, 0]-80, mk_obj.mk_pos[i, 1]]   # row needs substract padded height
    pnts = rs_obj.get_xyz(depth_frame, tar_pix)
    mk_obj.save_mk_pos_xyz(pnts)
    # vis
    cv2.imshow('realsense', mk_obj.mk_bbox_frame)
    key = cv2.waitKey(10)
    if key & 0xFF == ord('q') or key == 27 or frmCount >= nFrames:
      break
    elif key == ord('s'):
      cv2.imwrite(os.path.join(os.path.dirname(__file__), frm_save_path), mk_obj.mk_bbox_frame)
      print('marker frame saved')
    elapsed = time.time() - now
    time.sleep(freq-elapsed)
    frmCount += 1


if __name__ == "__main__":
  main()
