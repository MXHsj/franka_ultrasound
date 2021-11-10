#! /usr/bin/env python3
'''
experimenting deprojection in realsense SDK
'''
import csv
import numpy as np
from cv2 import cv2
from cv2 import aruco
from pyrealsense2 import pyrealsense2 as rs


dist_coeff = np.array([0.0430651, -0.1456001, 0.0, 0.0])
# dist_coeff = np.array([0.0, 0.0, 0.0, 0.0])

camera_matrix = np.array(   # lab realsense
    [[610.899, 0.0, 324.496],
     [0.0, 610.824, 234.984],
     [0.0, 0.0, 1.0]])

# Ran's realsense
# camera_matrix = np.array(   # Ran's realsense
#     [[610.899, 0.0, 326.496],
#      [0.0, 610.824, 250.984],
#      [0.0, 0.0, 1.0]])


def increase_brightness(img, value=30):
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  h, s, v = cv2.split(hsv)

  lim = 255 - value
  v[v > lim] = 255
  v[v <= lim] += value

  final_hsv = cv2.merge((h, s, v))
  img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
  return img


def detectFiducial(frame, fid_id=0):
  # marker detection
  frame = increase_brightness(frame, 20)
  # frame = cv2.blur(frame, (3, 3))
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

  # blur = cv2.blur(gray, (3, 3))
  aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
  parameters = aruco.DetectorParameters_create()
  corners, ids, _ = aruco.detectMarkers(
      gray, aruco_dict, parameters=parameters)
  marker_frame = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
  # pose estimation
  try:
    loc_marker = corners[np.where(ids == fid_id)[0][0]]
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
        loc_marker, 0.047, camera_matrix, dist_coeff)
    marker_frame = aruco.drawAxis(
        frame, camera_matrix, dist_coeff, rvecs, tvecs, 0.1)
    rmat = cv2.Rodrigues(rvecs)[0]
    # print(tvecs)
    tvec = np.transpose(tvecs)[:, 0, 0]
    # print("marker pose: \n", rmat[:, 2])
    UV = [loc_marker[0][:, 0].mean(), loc_marker[0][:, 1].mean()]
  except:
    rmat = -1*np.ones([3, 3])
    UV = [-1, -1]
    tvec = [-1, -1, -1]

  return marker_frame, UV, rmat[:, 2], tvec


def getNormalVector(p0, p1, p2):
  p1p0 = np.subtract(p1, p0)
  p2p0 = np.subtract(p2, p0)
  direction = np.cross(p1p0, p2p0)
  if direction[2] < 0:
    direction[0] = - direction[0]
    direction[1] = - direction[1]
    direction[2] = - direction[2]
  # magnitude = np.linalg.norm(direction)
  return direction


def getSurfaceNormal(point_x, point_y, point_z):
  # find normal vector of the plane P1P2P3P4
  P0 = [point_x[0], point_y[0], point_z[0]]
  P1 = [point_x[1], point_y[1], point_z[1]]
  P2 = [point_x[2], point_y[2], point_z[2]]
  P3 = [point_x[3], point_y[3], point_z[3]]
  P4 = [point_x[4], point_y[4], point_z[4]]
  P5 = [point_x[5], point_y[5], point_z[5]]
  P6 = [point_x[6], point_y[6], point_z[6]]
  P7 = [point_x[7], point_y[7], point_z[7]]
  P8 = [point_x[8], point_y[8], point_z[8]]
  # print(" P0: \n", P0, "\n P1: \n", P1, "\n P2: \n",
  #       P2, "\n P3: \n", P3, "\n P4: \n", P4)
  norm1 = getNormalVector(P0, P1, P2)
  norm2 = getNormalVector(P0, P2, P3)
  norm3 = getNormalVector(P0, P3, P4)
  norm4 = getNormalVector(P0, P4, P5)
  norm5 = getNormalVector(P0, P5, P6)
  norm6 = getNormalVector(P0, P6, P7)
  norm7 = getNormalVector(P0, P7, P8)
  norm8 = getNormalVector(P0, P1, P8)

  norm_vec = norm1+norm2+norm3+norm4+norm5+norm6+norm7+norm8
  norm_vec = - norm_vec  # make vector pointing inwards the surface
  norm_vec = norm_vec/np.linalg.norm(norm_vec)
  # print("surface normal: ", norm_vec)
  return norm_vec


def my_floor(a, precision=0):
  return np.round(a - 0.5 * 10**(-precision), precision)


def ROIshape(center, side=14):
  # square region
  col_vec = [center[0],
             center[0]-side/2,
             center[0]-side/2,
             center[0]-side/2,
             center[0],
             center[0]+side/2,
             center[0]+side/2,
             center[0]+side/2,
             center[0]]

  row_vec = [center[1],
             center[1]+side/2,
             center[1],
             center[1]-side/2,
             center[1]-side/2,
             center[1]-side/2,
             center[1],
             center[1]+side/2,
             center[1]+side/2]
  return col_vec, row_vec


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align = rs.align(rs.stream.color)

# depth filter
hole_filling = rs.hole_filling_filter()
spat_filter = rs.spatial_filter()       # reduce temporal noise
spat_filter.set_option(rs.option.filter_smooth_alpha, 1)
spat_filter.set_option(rs.option.filter_smooth_delta, 50)


isRecoding = False
sample_size = 100
sample_count = 0
print(" s->start recording \n e->end recording \n q->quit")
try:
  with open('../data/surface_normal.csv', 'w') as file_out:
    writer = csv.writer(file_out)
    while sample_count < sample_size:
      # Wait for a coherent pair of frames: depth and color
      frames = pipeline.wait_for_frames()

      # align depth to color frame
      aligned_frames = align.process(frames)
      depth_frame = aligned_frames.get_depth_frame()
      color_frame = aligned_frames.get_color_frame()

      # apply depth filters
      filtered = spat_filter.process(depth_frame)
      filtered = hole_filling.process(filtered)

      if not depth_frame or not color_frame:
        continue

      # Convert images to numpy arrays
      depth_image = np.asanyarray(filtered.get_data())
      color_image = np.asanyarray(color_frame.get_data())

      # detect fiducial marker
      marker_frame, markerUV, rvec, tvec = detectFiducial(color_image)
      if sum(markerUV) > 0:
        col_vec, row_vec = ROIshape(markerUV)
      else:
        col_vec, row_vec = ROIshape([320, 240])

      # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
      depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

      # get corresponding xyz from uv
      point_x = []
      point_y = []
      point_z = []
      depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
      # print(depth_intrin)

      # store 9 points
      for pnt in range(len(row_vec)):
        curr_col = int(col_vec[pnt])
        curr_row = int(row_vec[pnt])
        color_image = cv2.circle(
            marker_frame, (curr_col, curr_row), 1, (30, 90, 30), -1)
        depth_pixel = [curr_col, curr_row]
        depth_in_met = depth_frame.as_depth_frame().get_distance(curr_col, curr_row)
        # deprojection
        x = rs.rs2_deproject_pixel_to_point(
            depth_intrin, depth_pixel, depth_in_met)[0]
        y = rs.rs2_deproject_pixel_to_point(
            depth_intrin, depth_pixel, depth_in_met)[1]
        z = rs.rs2_deproject_pixel_to_point(
            depth_intrin, depth_pixel, depth_in_met)[2]
        if x != 0 and y != 0 and z != 0:
          point_x.append(x)
          point_y.append(y)
          point_z.append(z)
        else:       # points closer than 0.28m are bad points
          point_x.append(-1)
          point_y.append(-1)
          point_z.append(-1)

      # store suface normal
      norm_vec = getSurfaceNormal(point_x, point_y, point_z)
      point_x.append(my_floor(norm_vec[0], 3))
      point_y.append(my_floor(norm_vec[1], 3))
      point_z.append(my_floor(norm_vec[2], 3))

      # store ground truth rotation
      point_x.append(rvec[0])
      point_y.append(rvec[1])
      point_z.append(rvec[2])

      # store ground truth translation
      point_x.append(tvec[0])
      point_y.append(tvec[1])
      point_z.append(tvec[2])

      # write data into csv file
      if isRecoding:
        writer.writerow(point_x)
        writer.writerow(point_y)
        writer.writerow(point_z)
        sample_count += 1
        print("number of samples: ", sample_count)

      # Stack both images horizontally
      # images = np.vstack((color_image, depth_colormap))

      # Show images
      cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
      cv2.imshow('RealSense', color_image)

      key = cv2.waitKey(1)
      if key & 0xFF == ord('q') or key == 27:
        print('quit')
        break
      elif key == ord('s'):
        isRecoding = True
        print('start recoding data')
      elif key == ord('e'):
        isRecoding = False
        print('end recoding data')

finally:
  print("Finished.")
  # Stop streaming
  pipeline.stop()
  cv2.destroyAllWindows()
