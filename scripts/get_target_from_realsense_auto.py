#! /usr/bin/env python3
'''
select target using densepose
'''
import math
import csv
import numpy as np
from cv2 import cv2
from cv2 import aruco
from pyrealsense2 import pyrealsense2 as rs
import rospy
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16


# ---------------- densepose utilities ----------------
def getBodyPart(IUV, part_id=2):
  IUV_chest = np.zeros((IUV.shape[0], IUV.shape[1], IUV.shape[2]))
  torso_idx = np.where(IUV[:, :, 0] == part_id)
  IUV_chest[torso_idx] = IUV[torso_idx]
  return IUV_chest


def divide2region(IUV_chest, target_u, target_v):
  # input target u,v, output target row, col
  u2xy_pair = np.where(
      IUV_chest[:, :, 1] == target_u)    # find xy paris in u
  v2xy_pair = np.where(
      IUV_chest[:, :, 2] == target_v)    # find xy pairs in v

  rcand = list()
  ccand = list()

  u_x = u2xy_pair[1]
  u_y = u2xy_pair[0]
  v_x = v2xy_pair[1]
  v_y = v2xy_pair[0]

  # TODO: find x and y collaborately
  x_intersects = [x for x in u_x if x in v_x]
  y_intersects = [y for y in u_y if y in v_y]

  rcand = y_intersects
  ccand = x_intersects

  if len(rcand) > 0 and len(ccand) > 0:
    cen_col = int(np.mean(ccand))  # averaging col indicies
    cen_row = int(np.mean(rcand))  # averaging row indicies
  else:
    cen_col = -1
    cen_row = -1
  return [cen_col, cen_row]


def createMask(IUV_chest, frame):
  opacity = 0.35
  mask = np.zeros((IUV_chest.shape[0], IUV_chest.shape[1], IUV_chest.shape[2]))
  mask[:, :, 2] = IUV_chest[:, :, 0]*110
  overlay = cv2.addWeighted(mask, opacity, frame, 1.-opacity, -10., dtype=1)
  return overlay


# ---------------- tracking marker ----------------
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


# ---------------- ROS topics ----------------
def ee_callback(msg):
  EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
  global T_O_ee
  T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


def pub_key_cmd():
  if not rospy.is_shutdown():
    key_cmd_pub.publish(key_cmd_msg)


def pub_pose():
  if not rospy.is_shutdown():
    reg1_pub.publish(reg1_msg)
    reg2_pub.publish(reg2_msg)
    reg3_pub.publish(reg3_msg)
    reg4_pub.publish(reg4_msg)


def convert2base(T_cam_tar):
  # convert target from camera frame to base frame
  if T_O_ee is not None:
    T_O_cam = np.matmul(T_O_ee, T_ee_cam)
    T_O_tar = np.matmul(T_O_cam, T_cam_tar)
  else:
    T_O_tar = float('nan')*np.ones([4, 4])
    print("no robot info")
  return T_O_tar


# ---------------- surface normal ----------------
def drawVector(color_image, point_x, point_y, point_z):
  # camera intrinsics
  # lab realsense
  camera_matrix = np.array(
      [[610.899, 0.0, 324.496], [0.0, 610.824, 234.984], [0.0, 0.0, 1.0]])
  # Ran's realsense
  # camera_matrix = np.array(
  #     [[610.899, 0.0, 326.496], [0.0, 610.824, 250.984], [0.0, 0.0, 1.0]])

  Pc = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]  # world -> camera
  xyz2uv = np.matmul(camera_matrix, Pc)
  # draw surface normal vector on the output frame
  norm_vec = np.array([point_x[-1], point_y[-1], point_z[-1]])
  Pz_xyz = [point_x[0], point_y[0], point_z[0]] + 0.04*norm_vec
  Pz_xyz = np.append(Pz_xyz, 1.0)
  Pz_uv = np.matmul(xyz2uv, Pz_xyz)
  Pz_uv = [Pz_uv[0]/Pz_uv[2], Pz_uv[1]/Pz_uv[2]]
  P0_xyz = [point_x[0], point_y[0], point_z[0], 1.0]
  P0_uv = np.matmul(xyz2uv, P0_xyz)
  P0_uv = [P0_uv[0]/P0_uv[2], P0_uv[1]/P0_uv[2]]
  try:
    cv2.line(color_image,
             (int(P0_uv[0])-80, int(P0_uv[1])),
             (int(Pz_uv[0])-80, int(Pz_uv[1])),
             (200, 20, 20), 2)
  except Exception as e:
    pass
  return color_image


def calc_pose(point_x, point_y, point_z):
  if point_x is not None:
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
    # homogenuous transformation
    cam_tar = np.array([Vx, Vy, Vz, P0]).flatten()
    T_cam_tar = np.array([[cam_tar[0], cam_tar[3], cam_tar[6], cam_tar[9]],
                          [cam_tar[1], cam_tar[4], cam_tar[7], cam_tar[10]],
                          [cam_tar[2], cam_tar[5], cam_tar[8], cam_tar[11]],
                          [0.0, 0.0, 0.0, 1.0]])
    # entry pose w.r.t base
    dist_coeff = 0.075
    Pz = np.subtract(P0, [dist_coeff*Vzi for Vzi in Vz])
    T_cam_tar[0:3, 3] = Pz
    T_O_tar = convert2base(T_cam_tar)
  else:
    T_O_tar = float('nan')*np.ones([4, 4])

  # print(T_O_tar)
  tar_packed = np.transpose(np.array([T_O_tar[0], T_O_tar[1], T_O_tar[2]])).flatten()
  return tar_packed


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
  # if np.linalg.norm(norm_vec) != 0 else norm_vec
  return norm_vec


def my_floor(a, precision=0):
  return np.round(a - 0.5 * 10**(-precision), precision)


def ROIshape(center, edge=16):
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


# ---------------------constant transformations-----------------------------
# transformation from base to eef
# (data recorded at home pose, for debug purpose)
T_O_ee = np.array([[-0.0117, -0.9996, 0.0239, 0.0],
                   [-0.9989, 0.01278, 0.0435, 0.0],
                   [-0.0438, -0.0234, -0.9987, 0.0],
                   [0.3439, 0.0005, 0.4420, 1.0]]).transpose()
# T_O_ee = None

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

reg1_pub = rospy.Publisher('reg1_target', Float64MultiArray, queue_size=1)
reg1_msg = Float64MultiArray()
reg1_msg.data = float('nan')*np.ones([1, 12]).flatten()

reg2_pub = rospy.Publisher('reg2_target', Float64MultiArray, queue_size=1)
reg2_msg = Float64MultiArray()
reg2_msg.data = float('nan')*np.ones([1, 12]).flatten()

reg3_pub = rospy.Publisher('reg3_target', Float64MultiArray, queue_size=1)
reg3_msg = Float64MultiArray()
reg3_msg.data = float('nan')*np.ones([1, 12]).flatten()

reg4_pub = rospy.Publisher('reg4_target', Float64MultiArray, queue_size=1)
reg4_msg = Float64MultiArray()
reg4_msg.data = float('nan')*np.ones([1, 12]).flatten()

key_cmd_pub = rospy.Publisher('keyboard_cmd', Int16, queue_size=1)
key_cmd_msg = Int16()

rospy.Subscriber('franka_state_controller/franka_states', FrankaState, ee_callback)


def main():
  # densepose config
  save_path = '/home/xihan/DensePose/DensePoseData/image_buffer/incoming.png'
  load_path = '/home/xihan/DensePose/DensePoseData/infer_out/incoming_IUV.png'
  # target_u = [60, 100, 60, 100, 60, 100, 60, 100]
  target_u = [60, 100, 60, 100]
  # target_v = [150, 150, 192, 192, 105, 105, 58, 58]
  # target_v = [165, 165, 115, 115]
  target_v = [155, 155, 105, 105]
  inferred = None

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

  rospy.init_node('camera2target', anonymous=True)
  col_vec, row_vec = ROIshape([320, 240])
  point_x = []
  point_y = []
  point_z = []

  path2file = '/home/xihan/catkin_ws/src/robotic_ultrasound/franka-ultrasound/data/DP2normal.csv'
  file_out = open(path2file, 'w')
  writer = csv.writer(file_out)
  isRecoding = False
  print(" s->update targets  \n e->freeze targets \n g->go \n q->quit")
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
      # pad to square patch
      color_image = cv2.copyMakeBorder(color_image, 80, 80, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
      depth_image = cv2.copyMakeBorder(depth_image, 80, 80, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])

      # densepose inferrence
      cv2.imwrite(save_path, color_image)
      try:
        inferred = cv2.imread(load_path)
      except Exception as e:
        print('image loading error: '+str(e))

      if inferred is not None:
        IUV_chest = getBodyPart(inferred)
        for currRegionID in range(len(target_u)):
          curr_tar = [target_u[currRegionID], target_v[currRegionID]]
          target_pix = divide2region(IUV_chest, curr_tar[0], curr_tar[1])
          if np.isin(-1, target_pix, invert=True):
            col_vec, row_vec = ROIshape(target_pix)

            # get corresponding xyz from uv[-1, -1, -1]
            if isRecoding:
              point_x = []
              point_y = []
              point_z = []
              depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
              for pnt in range(len(row_vec)):
                curr_col = round(col_vec[pnt]+80)
                curr_row = round(row_vec[pnt])
                # color_image = cv2.circle(
                #     color_image, (curr_col-80, curr_row), 2, (30, 90, 30), -1)
                try:
                  depth_in_met = depth_frame.as_depth_frame().get_distance(curr_col, curr_row)
                  # deprojection
                  x = rs.rs2_deproject_pixel_to_point(depth_intrin, [curr_col, curr_row], depth_in_met)[0]
                  y = rs.rs2_deproject_pixel_to_point(depth_intrin, [curr_col, curr_row], depth_in_met)[1]
                  z = rs.rs2_deproject_pixel_to_point(depth_intrin, [curr_col, curr_row], depth_in_met)[2]
                  point_x.append(x)
                  point_y.append(y)
                  point_z.append(z)
                except:
                  continue

              cv2.circle(color_image, (target_pix[0], target_pix[1]), 15, (200, 200, 200), -1)
              cv2.putText(color_image, str(currRegionID+1), (target_pix[0], target_pix[1]),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (1, 100, 1), thickness=2)

              norm_vec = getSurfaceNormal(
                  point_x, point_y, point_z)
              point_x.append(my_floor(norm_vec[0], 3))
              point_y.append(my_floor(norm_vec[1], 3))
              point_z.append(my_floor(norm_vec[2], 3))
              tar_packed = calc_pose(point_x, point_y, point_z)

              # writer.writerow(point_x)
              # writer.writerow(point_y)
              # writer.writerow(point_z)
              # color_image = drawVector(
              #     color_image, point_x, point_y, point_z)

              if currRegionID == 0:
                reg1_msg.data = tar_packed
              elif currRegionID == 1:
                reg2_msg.data = tar_packed
              elif currRegionID == 2:
                reg3_msg.data = tar_packed
              elif currRegionID == 3:
                reg4_msg.data = tar_packed

        color_image = createMask(IUV_chest, color_image)
      else:
        pass

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
      else:
        key_cmd_msg.data = key

      pub_key_cmd()
      pub_pose()

  finally:
    # Stop streaming
    pipeline.stop()
    file_out.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
  main()
