#! /usr/bin/env python3
'''
record robot state T_O_ee, Wrench, isContact
'''
import os
import csv
import rospy
import numpy as np
from franka_msgs.msg import FrankaState
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped


def msg2matrix(raw_msg):
  # convert 12x1 message array to SE(3) matrix
  if np.isnan(raw_msg[0]):
    T = None
  else:
    T = np.array([[raw_msg[0], raw_msg[3], raw_msg[6], raw_msg[9]],
                  [raw_msg[1], raw_msg[4], raw_msg[7], raw_msg[10]],
                  [raw_msg[2], raw_msg[5], raw_msg[8], raw_msg[11]],
                  [0.0, 0.0, 0.0, 1.0]])
  return T


def ee_callback(msg):
  EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
  global T_O_ee
  T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


def force_callback(msg):
  global Fx, Fy, Fz
  Fx = msg.wrench.force.x
  Fy = msg.wrench.force.y
  Fz = msg.wrench.force.z


def reg1_tar_callback(msg):
  cam_tar = list(msg.data)
  global T_O_reg1
  T_O_reg1 = msg2matrix(cam_tar)


def reg2_tar_callback(msg):
  cam_tar = list(msg.data)
  global T_O_reg2
  T_O_reg2 = msg2matrix(cam_tar)


def reg3_tar_callback(msg):
  cam_tar = list(msg.data)
  global T_O_reg3
  T_O_reg3 = msg2matrix(cam_tar)


def reg4_tar_callback(msg):
  cam_tar = list(msg.data)
  global T_O_reg4
  T_O_reg4 = msg2matrix(cam_tar)


def key_cmd_callback(msg):
  global key_cmd
  key_cmd = msg.data


def isContact_callback(msg):
  global isContact
  isContact = msg.data


rospy.Subscriber('keyboard_cmd', Int16, key_cmd_callback)
rospy.Subscriber('franka_state_controller/franka_states', FrankaState, ee_callback)
rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, force_callback)
rospy.Subscriber('/isContact', Bool, isContact_callback)

rospy.Subscriber('reg1_target', Float64MultiArray, reg1_tar_callback)
rospy.Subscriber('reg2_target', Float64MultiArray, reg2_tar_callback)
rospy.Subscriber('reg3_target', Float64MultiArray, reg3_tar_callback)
rospy.Subscriber('reg4_target', Float64MultiArray, reg4_tar_callback)


# home pose
# T_O_ee = np.array([[-0.02406, -0.9997, -0.0001, 0.0],
#                    [-0.999, 0.02405, -0.0275, 0.0],
#                    [0.02751, -0.00055, -0.9996, 0.0],
#                    [0.26308, 0.025773, 0.2755, 1.0]]).transpose()

T_O_ee = np.array([[-0.0117, -0.9996, 0.0239, 0.0],
                   [-0.9989, 0.01278, 0.0435, 0.0],
                   [-0.0438, -0.0234, -0.9987, 0.0],
                   [0.3439, 0.0005, 0.4420, 1.0]]).transpose()
T_O_reg1 = np.nan*np.ones([4, 4])
T_O_reg2 = np.nan*np.ones([4, 4])
T_O_reg3 = np.nan*np.ones([4, 4])
T_O_reg4 = np.nan*np.ones([4, 4])
isContact = 0
key_cmd = -1


def main():
  rospy.init_node('robot_data_logger', anonymous=True)
  path2file = os.path.join(os.path.dirname(__file__), '../data/robot/robot_state_log.csv')
  file_out = open(path2file, 'w')
  writer = csv.writer(file_out)
  # writer.writerow(T_O_reg1.flatten())
  # writer.writerow(T_O_reg2.flatten())
  # writer.writerow(T_O_reg3.flatten())
  # writer.writerow(T_O_reg4.flatten())
  freq = 1
  rate = rospy.Rate(freq)
  print('start recording.')
  while not rospy.is_shutdown():
    data = T_O_ee.flatten()
    data = np.append(data, Fx)
    data = np.append(data, Fy)
    data = np.append(data, Fz)
    # data = np.append(data, isContact)
    writer.writerow(data)
    rate.sleep()
  print('\nend recording.')
  file_out.close()


if __name__ == "__main__":
  main()
