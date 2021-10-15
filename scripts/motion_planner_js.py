#! /usr/bin/env python3
'''
Name        : motion_planner_js
Author      : Xihan Ma
Version     : 
Description : control 6-DOF franka end-effector from flight joystick
'''
import math
import copy
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped


class Teleop:
  def __init__(self):
    rospy.init_node('franka_teleop_joy')
    # joystick buttons/axes
    self.ax0deadzone = rospy.get_param('~ax0deadzone', 0.03)
    self.ax1deadzone = rospy.get_param('~deadman_button', 0.06)
    self.ax4deadzone = rospy.get_param('~deadman_button', 0.095)
    self.cmd = None
    self.js = Joy()
    self.js.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.js.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # master & slave motion
    self.T_O_ee = np.array([[-0.0117, -0.9996, 0.0239, 0.0],
                            [-0.9989, 0.01278, 0.0435, 0.0],
                            [-0.0438, -0.0234, -0.9987, 0.0],
                            [0.3439, 0.0005, 0.4420, 1.0]]).transpose()
    self.curr_slave = Twist()
    self.last_slave = Twist()
    self.curr_master = Twist()
    self.last_master = Twist()
    self.d_slave = Twist()
    self.d_master = Twist()
    # tele operation states
    self.isContact = False          # automatic force compliant
    self.isAbort = False            # gradual stopping motion
    self.isEstop = False         # emergency-stop
    self.triggerState = False       # tranlation-rotation switch
    self.triggerStateOld = False
    self.wrench_slave = Wrench()
    self.wrench_slave_old = Wrench()
    self.force_des = 3.0  # max contact force [N]
    self.force_max = 8.0

    # ROS stuff
    cmd_pub = rospy.Publisher('franka_cmd_acc', Twist, queue_size=1)
    rospy.Subscriber("joy", Joy, self.js_callback)
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.ee_callback)
    rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.force_callback)
    self.freq = rospy.get_param('~hz', 600)
    rate = rospy.Rate(self.freq)

    while not rospy.is_shutdown():
      # update slave state
      self.curr_slave.linear.x = self.T_O_ee[0, 3]
      self.curr_slave.linear.y = self.T_O_ee[1, 3]
      self.curr_slave.linear.z = self.T_O_ee[2, 3]
      eul = self.rot2rpy(self.T_O_ee[:3, :3])
      eul[0] = eul[0] + 6.28 if eul[0] < 0 else eul[0]    # -pi->pi
      # print(eul[0])
      self.curr_slave.angular.x = eul[0]
      self.curr_slave.angular.y = eul[1]
      self.curr_slave.angular.z = eul[2]
      # slave velocity
      self.d_slave.linear.x = (self.curr_slave.linear.x-self.last_slave.linear.x)*self.freq
      self.d_slave.linear.y = (self.curr_slave.linear.y-self.last_slave.linear.y)*self.freq
      self.d_slave.linear.z = (self.curr_slave.linear.z-self.last_slave.linear.z)*self.freq
      self.d_slave.angular.x = (self.curr_slave.angular.x-self.last_slave.angular.x)*self.freq
      self.d_slave.angular.y = (self.curr_slave.angular.y-self.last_slave.angular.y)*self.freq
      self.d_slave.angular.z = (self.curr_slave.angular.z-self.last_slave.angular.z)*self.freq
      # master velocity
      self.d_master.linear.x = (self.curr_master.linear.x-self.last_master.linear.x)*self.freq
      self.d_master.linear.y = (self.curr_master.linear.y-self.last_master.linear.y)*self.freq
      self.d_master.linear.z = (self.curr_master.linear.z-self.last_master.linear.z)*self.freq
      self.d_master.angular.x = (self.curr_master.angular.x-self.last_master.angular.x)*self.freq
      self.d_master.angular.y = (self.curr_master.angular.y-self.last_master.angular.y)*self.freq
      self.d_master.angular.z = (self.curr_master.angular.z-self.last_master.angular.z)*self.freq
      # total force on end-effector
      self.force_sum = math.sqrt(self.wrench_slave.force.x**2 +
                                 self.wrench_slave.force.y**2 +
                                 self.wrench_slave.force.z**2)
      # publish command
      self.checkState(self.js)
      self.curr_master = self.mapjs2master(self.js)
      if self.isAbort or self.isEstop:
        self.cmd = self.stopMotion(self.cmd)
      else:
        # self.cmd = self.PDcontrol(self.js)
        self.cmd = self.direct_vel_mapping(self.js)
      if self.cmd:
        cmd_pub.publish(self.cmd)
      self.last_slave = copy.deepcopy(self.curr_slave)
      self.last_master = copy.deepcopy(self.curr_master)
      rate.sleep()

  def checkState(self, data):
    # switch between contact & non-contact mode
    if data.buttons[10]:
      if data.axes[0]+data.axes[1]+data.axes[4] < 0.05:
        self.isContact = not self.isContact
        print("Contact mode: ", self.isContact)
      else:
        print("Only switch contact mode at neutral position")

    # abort if e-stop pressed
    self.isEstop = data.buttons[1]

    # abort if trigger pressed at non-neutral position
    self.triggerState = data.buttons[0]
    if self.triggerState is not self.triggerStateOld:
      if abs(data.axes[0]) > 0.1 or abs(data.axes[1]) > 0.1:
        print("Abort! move to neutral position!")
        self.isAbort = True
    self.triggerStateOld = self.triggerState

    # resume motion by putting js to neutral
    if self.isAbort:
      if abs(data.axes[0]) < 0.05 and abs(data.axes[1]) < 0.05:
        print("Ready to move!")
        self.isAbort = False

    # check external force on eef
    if self.force_sum > self.force_max:
      print("exceeding max external force")

  def mapjs2master(self, data):
    '''map joystick input to desired eef pose'''
    temp = Twist()
    sLin = [0.008, 0.008, 0.006]   # [x, y, z] [0.008, 0.008, 0.006]
    sAng = [0.008, 0.008, 0.008]   # [x, y, z] [0.008, 0.008, 0.01]
    stiff = [6e-4, 6e-4, 7e-4]    # [x, y, z] [6e-4, 6e-4, 7e-4]
    dampz = 1.8e-6
    Vz = self.T_O_ee[:3, 2]		# approach vector
    force_error = self.force_des-self.wrench_slave.force.z
    force_error_d = (self.wrench_slave_old.force.z-self.wrench_slave.force.z)*self.freq

    if self.triggerState:
      # control angular position
      if self.isContact:
        temp.angular.x = self.curr_slave.angular.x + \
            sAng[0]*data.axes[0]*(self.force_sum < self.force_max)
        temp.angular.y = self.curr_slave.angular.y + \
            sAng[1]*data.axes[1]*(self.force_sum < self.force_max)
        temp.angular.z = self.curr_slave.angular.z + \
            sAng[2]*data.axes[4]*(self.force_sum < self.force_max)
      else:
        temp.angular.x = self.curr_slave.angular.x + \
            sAng[0]*data.axes[0]
        temp.angular.y = self.curr_slave.angular.y + \
            sAng[1]*data.axes[1]
        temp.angular.z = self.curr_slave.angular.z + \
            sAng[2]*data.axes[4]
      temp.linear.x = self.curr_slave.linear.x
      temp.linear.y = self.curr_slave.linear.y
      temp.linear.z = self.curr_slave.linear.z
    elif not self.triggerState:
      # control linear position
      if self.isContact:
        temp.linear.x = self.curr_slave.linear.x + \
            (sLin[0]/5*data.axes[1] + stiff[0]*force_error*Vz[0])
        temp.linear.y = self.curr_slave.linear.y - \
            (sLin[1]/5*data.axes[0] + stiff[1]*force_error*Vz[1])
        temp.linear.z = self.curr_slave.linear.z + \
            (stiff[2]*force_error+dampz*force_error_d)*Vz[2]
        print(temp.linear.z)
      else:
        temp.linear.x = self.curr_slave.linear.x + \
            (sLin[0]*data.axes[1])
        temp.linear.y = self.curr_slave.linear.y - \
            (sLin[1]*data.axes[0])
        temp.linear.z = self.curr_slave.linear.z - \
            (sLin[2]*data.axes[4])
      temp.angular.x = self.curr_slave.angular.x
      temp.angular.y = self.curr_slave.angular.y
      temp.angular.z = self.curr_slave.angular.z
    return temp

  def PDcontrol(self, js_msg):
    '''
    acceleration command: u = Kp*(Xm-Xs) + Kd*(dXm-dXs)
    '''
    # TODO: separate remove residual function
    u = Twist()
    # linear
    # kp = 1.3, kd = 0.2
    u.linear.x = \
        1.2*(self.curr_master.linear.x - self.curr_slave.linear.x) \
        # + 0.2*(self.d_master.linear.x - self.d_slave.linear.x)
    # kp = 1.4, kd = 0.2
    u.linear.y = \
        1.2*(self.curr_master.linear.y - self.curr_slave.linear.y) \
        # + 0.2*(self.d_master.linear.y - self.d_slave.linear.y)
    # kp = 1.0, kd = 0.2
    u.linear.z = \
        1.5*(self.curr_master.linear.z - self.curr_slave.linear.z) \
        # + 0.2*(self.d_master.linear.z - self.d_slave.linear.z)
    # angular
    # kp = 2.8, kd = 0.2
    u.angular.x = 0.0 if abs(js_msg.axes[0]) < self.ax0deadzone else \
        2.8*(self.curr_master.angular.x - self.curr_slave.angular.x) \
        # + 0.2*(self.d_master.angular.x - self.d_slave.angular.x)
    # print(u.angular.x)
    # kp = 3.0, kd = 0.2
    u.angular.y = 0.0 if abs(js_msg.axes[1]) < self.ax1deadzone else \
        3.0*(self.curr_master.angular.y - self.curr_slave.angular.y) \
        # + 0.2*(self.d_master.angular.y - self.d_slave.angular.y)
    # kp = 4.2, kd = 0.4
    u.angular.z = 0.0 if abs(js_msg.axes[4]) < self.ax4deadzone else \
        4.0*(self.curr_master.angular.z - self.curr_slave.angular.z) \
        # + 0.4*(self.d_master.angular.z - self.d_slave.angular.z)

    # get rid of small motion
    u.linear.x = 0.0 if abs(u.linear.x) < 1e-7 else u.linear.x
    u.linear.y = 0.0 if abs(u.linear.y) < 1e-7 else u.linear.y
    u.linear.z = 0.0 if abs(u.linear.z) < 1e-7 else u.linear.z
    return u

  def direct_vel_mapping(self, js_msg):
    lin_acc_max = 13.0*0.018
    ang_acc_max = 25.0*0.018
    lin_acc_o2 = -lin_acc_max
    lin_acc_o1 = lin_acc_max
    lin_vel_o3 = (1/3)*lin_acc_o2
    lin_vel_o2 = (1/2)*lin_acc_o1
    u = Twist()
    # linear
    u.linear.x = math.copysign(
        lin_vel_o3*abs(js_msg.axes[1])**3 + lin_vel_o2*abs(js_msg.axes[1])**2, js_msg.axes[1])
    u.linear.y = math.copysign(
        lin_vel_o3*abs(js_msg.axes[0])**3 + lin_vel_o2*abs(js_msg.axes[0])**2, js_msg.axes[0])
    # angular
    # u.angular.x = -4.1667*js_msg.axes[0]**3 + 6.25*js_msg.axes[0]**2 if js_msg.axes[0] > 0 else \
    #     -(-4.1667*js_msg.axes[0]**3 + 6.25*js_msg.axes[0]**2)
    # u.angular.y = -4.1667*js_msg.axes[1]**3 + 6.25*js_msg.axes[1]**2 if js_msg.axes[1] > 0 else \
    #     -(-4.1667*js_msg.axes[1]**3 + 6.25*js_msg.axes[1]**2)
    return u

  def stopMotion(self, cmd):
    # gradually decrease acceleration to 0
    incre = 2e-4
    tol = 1e-5
    cmd.linear.x -= math.copysign(incre, cmd.linear.x) \
        if abs(cmd.linear.x) > tol else 0.0
    cmd.linear.y -= math.copysign(incre, cmd.linear.y) \
        if abs(cmd.linear.y) > tol else 0.0
    cmd.linear.z -= math.copysign(incre, cmd.linear.z) \
        if abs(cmd.linear.z) > tol else 0.0
    cmd.angular.x -= math.copysign(incre, cmd.angular.x) \
        if abs(cmd.angular.x) > tol else 0.0
    cmd.angular.y -= math.copysign(incre, cmd.angular.y) \
        if abs(cmd.angular.y) > tol else 0.0
    cmd.angular.z -= math.copysign(incre, cmd.angular.z) \
        if abs(cmd.angular.z) > tol else 0.0
    return cmd

  def js_callback(self, js_msg):
    self.js = js_msg

  def ee_callback(self, ee_msg):
    EE_pos = ee_msg.O_T_EE_d  # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12],
                            EE_pos[12:16]]).transpose()

  def force_callback(self, FT_msg):
    self.wrench_slave_old = self.wrench_slave
    self.wrench_slave.force.x = 0.0 if FT_msg.wrench.force.x < 0 \
        else FT_msg.wrench.force.x
    self.wrench_slave.force.y = 0.0 if FT_msg.wrench.force.y < 0 \
        else FT_msg.wrench.force.y
    self.wrench_slave.force.z = 0.0 if FT_msg.wrench.force.z < 0 \
        else FT_msg.wrench.force.z

  def rot2rpy(self, R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
      x = math.atan2(R[2, 1], R[2, 2])
      y = math.atan2(-R[2, 0], sy)
      z = math.atan2(R[1, 0], R[0, 0])
    else:
      x = math.atan2(-R[1, 2], R[1, 1])
      y = math.atan2(-R[2, 0], sy)
      z = 0
    return np.array([x, y, z])


if __name__ == "__main__":
  Teleop()
