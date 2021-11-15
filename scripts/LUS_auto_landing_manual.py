#! /usr/bin/env python3
'''
integrate teleop
'''
import math
import rospy
import numpy as np
from cv2 import cv2
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from pyrealsense2 import pyrealsense2 as rs


def ee_callback(msg):
  EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
  global T_O_ee
  T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


def js_callback(msg):
  global cmd_acc_msg
  cmd_acc_msg = msg


def convert2base(T_cam_tar):
  # convert target from camera frame to base frame
  if T_O_ee is not None:
    T_O_cam = np.matmul(T_O_ee, T_ee_cam)
    T_O_tar = np.matmul(T_O_cam, T_cam_tar)
  else:
    T_O_tar = float('nan')*np.ones([4, 4])
    print("no robot info")
  return T_O_tar


def calc_pose(point_x, point_y, point_z):
  if point_x is not None:
    # z component
    P0 = [point_x[0], point_y[0], point_z[0]]
    Vz = [point_x[-1], point_y[-1], point_z[-1]]
    # x component
    # xx = math.cos(math.pi/6)                            # 1.0
    # yx = math.sin(math.pi/6)*math.cos(math.pi/8)        # 0.0
    xx = 1.0
    yx = 0.0
    zx = -(Vz[1]*(yx-P0[1])+Vz[0]*(xx-P0[0]))/Vz[2]+P0[2]
    Vx = np.subtract([xx, yx, zx], P0)
    Vx = my_floor(Vx/np.linalg.norm(Vx), 3)
    # y component
    Vy = np.cross(Vz, Vx)
    Vy = my_floor(Vy/np.linalg.norm(Vy), 3)
    # homogenuous transformation
    cam_tar = np.array([Vx, Vy, Vz, P0]).flatten()
    T_cam_tar = np.array([[cam_tar[0], cam_tar[3], cam_tar[6], cam_tar[9]],
                          [cam_tar[1], cam_tar[4], cam_tar[7], cam_tar[10]],
                          [cam_tar[2], cam_tar[5], cam_tar[8], cam_tar[11]],
                          [0.0, 0.0, 0.0, 1.0]])
    # entry pose w.r.t base
    dist_coeff = 0.05      # 0.075
    Pz = np.subtract(P0, [dist_coeff*Vzi for Vzi in Vz])
    T_cam_tar[0:3, 3] = Pz
    T_O_tar = convert2base(T_cam_tar)
  else:
    T_O_tar = float('nan')*np.ones([4, 4])
  print('T_O_tar \n', T_O_tar)
  tar_packed = T_O_tar[:3, :4].transpose().flatten()
  return tar_packed
  # return T_O_tar.flatten()


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


def my_floor(a, precision=0):
  return np.round(a - 0.5 * 10**(-precision), precision)


# ---------------------constant transformations-----------------------------
# transformation from base to eef
# (data recorded at home pose, for debug purpose)
T_O_ee = np.array([[-0.02406, -0.9997, -0.0001, 0.0],
                   [-0.999, 0.02405, -0.0275, 0.0],
                   [0.02751, -0.00055, -0.9996, 0.0],
                   [0.26308, 0.025773, 0.2755, 1.0]]).transpose()
# T_O_ee = None
# home position
T_O_home = np.array([[0.06194, -0.99796, -0.01534, 0.0],
                     [-0.99806, -0.06200, 0.00357, 0.0],
                     [-0.00451, 0.01509, -0.99987, 0.0],
                     [0.24109, 0.04896, 0.27874, 1.0]]).transpose()

# transformation from custom eef to camera [m]
eef_id = 1
if eef_id == 0:
  # Jakub eef
  T_ee_cam = np.array([[1.000, 0.0, 0.0, -0.0175],
                      [0.0, 0.9239, -0.3827, -0.0886180],
                      [0.0, 0.3827, 0.9239, -0.3233572],
                      [0.0, 0.0, 0.0, 1.0]])
elif eef_id == 1:
  # Clarius eef
  T_ee_cam = np.array([[0.0, 0.0, 0.3827, 0.0886],
                      [1.0, -0.9239, 0.0, -0.0175],
                      [0.0, 0.3827, 0.9239, -0.2889],
                      [0.0, 0.0, 0.0, 1.0]])
# --------------------------------------------------------------------------

rospy.Subscriber('franka_state_controller/franka_states',
                 FrankaState, ee_callback)
rospy.Subscriber('cmd_js', Twist, js_callback)

scan_tar_pub = rospy.Publisher('scan_targets', Float64MultiArray, queue_size=1)
cmd_pos_pub = rospy.Publisher('franka_cmd_pos', Float64MultiArray, queue_size=1)
cmd_acc_pub = rospy.Publisher('franka_cmd_acc', Twist, queue_size=1)
contact_mode_pub = rospy.Publisher('isContact', Bool, queue_size=1)

scan_tar_msg = Float64MultiArray()
cmd_pos_msg = Float64MultiArray()
cmd_acc_msg = Twist()
contact_mode_msg = Bool()


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
  pix_tar = [320, 240]

  # initialize ros node
  rospy.init_node('LUS_auto_landing_manual', anonymous=True)

  max_num_reg = 4
  reg_data = float('nan')*np.ones([max_num_reg, 12])
  scan_tar_msg.data = reg_data.flatten()

  tar_count = 1
  tar_togo = 1
  contact_mode_msg.data = False
  curr_goal = T_O_ee[:3, :4]
  op_mode = 'tele'
  print("-----------camera commands---------")
  print(" p->publish \n r->reset \n q->quit")
  print("-----------teleop commands---------")
  print(' g->go to next target \n h->go to home pose \n')

  point_x = [0]*10
  point_y = [0]*10
  point_z = [0]*10
  rate = rospy.Rate(80)
  try:
    while True:
      col_vec, row_vec = ROIshape(pix_tar)
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
      # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
      #     depth_image, alpha=0.03), cv2.COLORMAP_JET)

      # get corresponding xyz from uv[-1, -1, -1]

      depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
      for pnt in range(len(row_vec)):
        curr_col = round(col_vec[pnt])
        curr_row = round(row_vec[pnt])
        color_image = cv2.circle(
            color_image, (curr_col, curr_row), 2, (30, 90, 30), -1)
        depth_pixel = [curr_col, curr_row]
        depth_in_met = depth_frame.as_depth_frame().get_distance(curr_col, curr_row)
        # deprojection
        x = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_in_met)[0]
        y = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_in_met)[1]
        z = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_in_met)[2]
        point_x[pnt] = x
        point_y[pnt] = y
        point_z[pnt] = z

      norm_vec = getSurfaceNormal(point_x, point_y, point_z)
      point_x[-1] = my_floor(norm_vec[0], 3)
      point_y[-1] = my_floor(norm_vec[1], 3)
      point_z[-1] = my_floor(norm_vec[2], 3)

      # Stack both images horizontally
      # images = np.vstack((color_image, depth_colormap))

      # Show images
      cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
      cv2.imshow('RealSense', color_image)

      pose_err = np.subtract(curr_goal, T_O_ee[:3, :4])
      trans_error = pose_err[:, -1]
      rot_error = pose_err[:, :3].flatten()
      isReachedTrans = True if all([abs(err) < 0.0035 for err in trans_error]) else False
      isReachedRot = True if all([abs(err) < 0.08 for err in rot_error]) else False
      if isReachedRot and isReachedTrans and op_mode == 'auto':
        # if op_mode == 'auto':
        print("arrive at target ", tar_togo-1, " switch to manual")
        op_mode = 'tele'
        contact_mode_msg.data = True
        # elif op_mode == 'home':
        #     print("arrive home")
        #     op_mode = 'tele'

      # keyboard control
      key = cv2.waitKey(1)
      if key & 0xFF == ord('q') or key == 27:
        print('quit')
        break
      elif key == ord('p'):
        print('publish target ', tar_togo)
        tar_packed = calc_pose(point_x, point_y, point_z)
        reg_data[tar_count-1] = tar_packed
        scan_tar_msg.data = reg_data.flatten()
        tar_count = tar_count + 1 if tar_count < max_num_reg else 1
      elif key == ord('r'):
        print('reset all targets')
        reg_data = float('nan')*np.ones([max_num_reg, 12])
        scan_tar_msg.data = reg_data.flatten()
        tar_count = 1
      elif key == ord('g'):   # go to the next target
        op_mode = 'auto' if op_mode != 'auto' else op_mode
        contact_mode_msg.data = False
        if np.isnan(reg_data[tar_togo-1]).any():
          print('not a valid pose')
        else:
          curr_goal = reg_data[tar_togo-1].reshape(4, 3).transpose()
          print('go to target ', tar_togo, ': \n', curr_goal)
          cmd_pos_msg.data = reg_data[tar_togo-1]
          tar_togo = tar_togo + 1 if tar_togo < max_num_reg else 1
      elif key == ord('h'):   # go back to home pose
        op_mode = 'home' if op_mode != 'home' else op_mode
        contact_mode_msg.data = False
        tar_togo = 1   # set target to the first
        print('going home')
        cmd_pos_msg.data = T_O_home[:3, :4].transpose().flatten()

      elif key == ord('w'):
        pix_tar[1] = pix_tar[1] - 10 if pix_tar[1]-20 > 0 else pix_tar[1]
        print('move up', pix_tar)
      elif key == ord('a'):
        pix_tar[0] = pix_tar[0] - 10 if pix_tar[0]-20 > 0 else pix_tar[0]
        print('move left', pix_tar)
      elif key == ord('s'):
        pix_tar[1] = pix_tar[1] + 10 if pix_tar[1]+20 < 480 else pix_tar[1]
        print('move down', pix_tar)
      elif key == ord('d'):
        print('move right', pix_tar)
        pix_tar[0] = pix_tar[0] + 10 if pix_tar[0]+20 < 640 else pix_tar[0]

      if op_mode == 'auto' or op_mode == 'home':
        # cmd_acc_msg.linear.x = float('nan')
        # cmd_acc_msg.linear.y = float('nan')
        # cmd_acc_msg.linear.z = float('nan')
        # cmd_acc_msg.angular.x = float('nan')
        # cmd_acc_msg.angular.y = float('nan')
        # cmd_acc_msg.angular.z = float('nan')
        cmd_pos_pub.publish(cmd_pos_msg)
      elif op_mode == 'tele':
        # cmd_pos_msg.data = float('nan')*np.ones([max_num_reg*12])
        cmd_acc_pub.publish(cmd_acc_msg)

      scan_tar_pub.publish(scan_tar_msg)
      contact_mode_pub.publish(contact_mode_msg)

      rate.sleep()

  finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
  main()
