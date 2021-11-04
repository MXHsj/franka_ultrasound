#! /usr/bin/env python3
'''
publish target pose w.r.t realsense
'''
import numpy as np
from cv2 import cv2
import math
import rospy
from std_msgs.msg import Float64MultiArray
from pyrealsense2 import pyrealsense2 as rs


tar_pub = rospy.Publisher('cam_target', Float64MultiArray, queue_size=1)
tar_msg = Float64MultiArray()


def pub_pos(point_x, point_y, point_z):
  if len(point_x) > 0:        # when start streaming data
    # z component
    P0 = [point_x[0], point_y[0], point_z[0]]
    Vz = [point_x[-1], point_y[-1], point_z[-1]]
    # x component
    xx = math.cos(math.pi/6)                            # 1.0
    yx = math.sin(math.pi/6)*math.cos(math.pi/8)        # 0.0
    zx = -(Vz[1]*(yx-P0[1])+Vz[0]*(xx-P0[0]))/Vz[2]+P0[2]
    Vx = np.subtract([xx, yx, zx], P0)
    Vx = my_floor(Vx/np.linalg.norm(Vx), 3)
    # y component
    Vy = np.cross(Vz, Vx)
    Vy = my_floor(Vy/np.linalg.norm(Vy), 3)
    tar_pose = np.array([Vx, Vy, Vz, P0]).flatten()   # 4x4 target
    T_cam_tar = np.array([[tar_pose[0], tar_pose[3], tar_pose[6], tar_pose[9]],
                          [tar_pose[1], tar_pose[4], tar_pose[7], tar_pose[10]],
                          [tar_pose[2], tar_pose[5], tar_pose[8], tar_pose[11]],
                          [0.0, 0.0, 0.0, 1.0]])
  else:
    T_cam_tar = np.array([[-1.0, -1.0, -1.0, -1.0], [-1.0, -1.0, -1.0, -1.0],
                          [-1.0, -1.0, -1.0, -1.0], [-1.0, -1.0, -1.0, -1.0]])

  # print("camera target: \n", T_cam_tar)

  tar_packed = np.transpose(
      np.array([T_cam_tar[0], T_cam_tar[1], T_cam_tar[2]])).flatten()
  tar_msg.data = tar_packed
  if not rospy.is_shutdown():
    tar_pub.publish(tar_msg)


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

  # averaging + normalization
  norm_vec = norm1+norm2+norm3+norm4+norm5+norm6+norm7+norm8
  norm_vec = norm_vec/np.linalg.norm(norm_vec)
  return norm_vec


def my_floor(a, precision=0):
  return np.round(a - 0.5 * 10**(-precision), precision)


def ROIshape(center, edge=12):
  # square region
  col_vec = [center[0],
             center[0]-edge/2,
             center[0]-edge/2,
             center[0]-edge/2,
             center[0],
             center[0]+edge/2,
             center[0]+edge/2,
             center[0]+edge/2,
             center[0]]

  row_vec = [center[1],
             center[1]+edge/2,
             center[1],
             center[1]-edge/2,
             center[1]-edge/2,
             center[1]-edge/2,
             center[1],
             center[1]+edge/2,
             center[1]+edge/2]
  return col_vec, row_vec


def main():
  # Configure depth and color streams
  pipeline = rs.pipeline()
  config = rs.config()
  config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
  config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

  # Start streaming
  # profile = pipeline.start(config)
  # depth_sensor = profile.get_device().first_depth_sensor()
  # depth_scale = depth_sensor.get_depth_scale()
  pipeline.start(config)
  align = rs.align(rs.stream.color)

  # depth filter
  hole_filling = rs.hole_filling_filter()
  spat_filter = rs.spatial_filter()       # reduce temporal noise
  spat_filter.set_option(rs.option.filter_smooth_alpha, 1)
  spat_filter.set_option(rs.option.filter_smooth_delta, 50)

  # default target in pixel
  col_vec, row_vec = ROIshape([320, 240])

  # initialize ros node
  rospy.init_node('camera2target', anonymous=True)
  point_x = []
  point_y = []
  point_z = []

  isRecoding = False
  print(" s->update data \n e->freeze data \n q->quit")
  try:
    while True:
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

      # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
      depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
          depth_image, alpha=0.03), cv2.COLORMAP_JET)

      # get corresponding xyz from uv[-1, -1, -1]
      if isRecoding:
        point_x = []
        point_y = []
        point_z = []
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        for pnt in range(len(row_vec)):
          curr_col = round(col_vec[pnt])
          curr_row = round(row_vec[pnt])
          color_image = cv2.circle(
              color_image, (curr_col, curr_row), 2, (30, 90, 30), -1)
          depth_pixel = [curr_col, curr_row]
          depth_in_met = depth_frame.as_depth_frame().get_distance(curr_col, curr_row)
          # deprojection
          x = rs.rs2_deproject_pixel_to_point(
              depth_intrin, depth_pixel, depth_in_met)[0]
          y = rs.rs2_deproject_pixel_to_point(
              depth_intrin, depth_pixel, depth_in_met)[1]
          z = rs.rs2_deproject_pixel_to_point(
              depth_intrin, depth_pixel, depth_in_met)[2]
          point_x.append(x)
          point_y.append(y)
          point_z.append(z)

        norm_vec = getSurfaceNormal(point_x, point_y, point_z)
        point_x.append(my_floor(norm_vec[0], 3))
        point_y.append(my_floor(norm_vec[1], 3))
        point_z.append(my_floor(norm_vec[2], 3))

      pub_pos(point_x, point_y, point_z)

      # Stack both images horizontally
      # images = np.vstack((color_image, depth_colormap))

      # Show images
      cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
      cv2.imshow('RealSense', color_image)

      # keyboard control
      key = cv2.waitKey(1)
      if key & 0xFF == ord('q') or key == 27:
        print('quit')
        break
      elif key == ord('s'):
        isRecoding = True
        print('update data')
      elif key == ord('e'):
        isRecoding = False
        print('freeze data')

  finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
  main()
