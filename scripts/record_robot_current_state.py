#! /usr/bin/env python3
'''
record robot state T_O_ee, Wrench
'''
import os
import csv
import rospy
import numpy as np
from franka_msgs.msg import FrankaState
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


if __name__ == "__main__":
  rospy.Subscriber('franka_state_controller/franka_states', FrankaState, ee_callback)
  rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, force_callback)

  T_O_ee = None

  rospy.init_node('robot_data_logger', anonymous=True)
  file_path = os.path.join(os.path.dirname(__file__), '../data/robot/robot_state.csv')
  file_out = open(file_path, 'w')
  writer = csv.writer(file_out)
  freq = 20
  rate = rospy.Rate(freq)
  print('connecting to robot ...')
  while not rospy.is_shutdown():
    if T_O_ee is not None:
      break
    rate.sleep()
  data = T_O_ee.flatten()
  data = np.append(data, Fx)
  data = np.append(data, Fy)
  data = np.append(data, Fz)
  print('current pose: \n', T_O_ee)
  print('current force on eef" \n', Fx, 'N ', Fy, 'N ', Fz, 'N ')
  writer.writerow(data)
  print('robot pose written to file ', file_path)
  file_out.close()
