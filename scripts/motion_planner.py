#! /usr/bin/env python3
'''
motion planner for lung ultrasound
'''
import numpy as np
import rospy
import math
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Int16


def eulerAnglesToRotationMatrix(theta):
    # Calculates Rotation Matrix given euler angles.
    R_x = np.array([[1,         0,                  0],
                    [0,         math.cos(theta[0]), -math.sin(theta[0])],
                    [0,         math.sin(theta[0]), math.cos(theta[0])]])
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])],
                    [0,                     1,      0],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])]])
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]])
    R = np.matmul(R_z, np.matmul(R_y, R_x))
    return R


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
    T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12],
                       EE_pos[12:16]]).transpose()


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


def pub_pos(T_O_tar):
    tar_msg = Float64MultiArray()
    if T_O_tar is not None:
        tar_packed = np.transpose(
            np.array([T_O_tar[0], T_O_tar[1], T_O_tar[2]])).flatten()
    else:
        tar_packed = float('nan')*np.ones([12, 1]).flatten()
    tar_msg.data = tar_packed
    if not rospy.is_shutdown():
        tar_pub.publish(tar_msg)


def pub_contact_mode(contactFlag=False):
    op_msg = Bool()
    op_msg.data = contactFlag
    if not rospy.is_shutdown():
        op_pub.publish(op_msg)


rospy.Subscriber('keyboard_cmd', Int16, key_cmd_callback)
rospy.Subscriber('franka_state_controller/franka_states',
                 FrankaState, ee_callback)

rospy.Subscriber('reg1_target', Float64MultiArray, reg1_tar_callback)
rospy.Subscriber('reg2_target', Float64MultiArray, reg2_tar_callback)
rospy.Subscriber('reg3_target', Float64MultiArray, reg3_tar_callback)
rospy.Subscriber('reg4_target', Float64MultiArray, reg4_tar_callback)

tar_pub = rospy.Publisher('target_pose', Float64MultiArray, queue_size=1)
op_pub = rospy.Publisher('isContact', Bool, queue_size=1)


# transformation from base to eef
# (data recorded at home pose, for debug purpose)
T_O_ee = np.array([[-0.0117, -0.9996, 0.0239, 0.0],
                   [-0.9989, 0.01278, 0.0435, 0.0],
                   [-0.0438, -0.0234, -0.9987, 0.0],
                   [0.3439, 0.0005, 0.4420, 1.0]]).transpose()
# T_O_ee = None
T_O_reg1 = None
T_O_reg2 = None
T_O_reg3 = None
T_O_reg4 = None
key_cmd = -1


def main():
    global T_O_ee
    isContact = False
    isTeleop = False
    isExecute = False
    isFirstContact = True
    isRelease = False
    T_O_ee_last = T_O_ee
    curr_target = None
    snapshotTime = 0.0
    # RCM control params
    roll_d = 0.0
    pitch_d = 0.0
    yaw_d = -math.pi/2
    R_desired = 0.2
    x_obj = 0.22
    z_obj = 0.12

    rospy.init_node('waypoints_generator', anonymous=True)
    target_idx = 0      # should be 0
    num_regions = 3
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
        if key_cmd == ord('t'):
            isTeleop = not isTeleop
            isExecute = False
            print("tele-op mode: ", isTeleop)
        elif key_cmd == ord('g'):
            isTeleop = False
            isExecute = not isExecute
            print("execution mode: ", isExecute)

        # RCM tele-operation mode
        if not isExecute and isTeleop:
            isContact = False
            # print('RCM mode')
            x_d = x_obj
            y_d = R_desired*math.sin(roll_d)
            z_d = z_obj + R_desired*math.cos(roll_d)
            if key_cmd == ord('j'):
                roll_d -= 0.05    # rotate CW around x_base
                # print("x:", np.round(x_d, 4), "y:", np.round(y_d, 4), "z:", np.round(z_d, 4),
                #       "r:", np.round(roll_d, 4), "p:", np.round(pitch_d, 4), "y:", np.round(yaw_d, 4))
            if key_cmd == ord('l'):
                roll_d += 0.05    # rotate CCW around x_base
                # print("x:", np.round(x_d, 4), "y:", np.round(y_d, 4), "z:", np.round(z_d, 4),
                #       "r:", np.round(roll_d, 4), "p:", np.round(pitch_d, 4), "y:", np.round(yaw_d, 4))
            Rot = eulerAnglesToRotationMatrix([roll_d, pitch_d, yaw_d])
            curr_target = np.array([[Rot[0, 0], Rot[0, 1], Rot[0, 2], x_d],
                                    [Rot[1, 0], Rot[1, 1], Rot[1, 2], y_d],
                                    [Rot[2, 0], Rot[2, 1], Rot[2, 2], z_d]])
            print(curr_target)

        # sequentially travel to waypoints
        if isExecute and not isTeleop:
            waypoints = [T_O_reg1, T_O_reg2, T_O_reg3, T_O_reg4]
            curr_target = waypoints[target_idx]
            if curr_target is None:
                target_idx = target_idx + 1 if target_idx < num_regions else target_idx
                print("target {} non exist".format(target_idx+1))
            else:
                T_O_ee_last = T_O_ee

                T_error = np.subtract(curr_target, T_O_ee_last)
                trans_error = T_error[0:3, 3]
                rot_error = T_error[0:3, 0:3].flatten()

                isReachedTrans = True if sum(
                    [abs(err) < 0.002 for err in trans_error]) == len(trans_error) else False
                isReachedRot = True if sum(
                    [abs(err) < 0.03 for err in rot_error]) == len(rot_error) else False

                # print("trans error: \n", trans_error)
                # print("rot error: \n", rot_error)
                # print("trans arrival: ", isReachedTrans,
                #       "\nrot arrival: ", isReachedRot)

                if isReachedRot and isReachedTrans:
                    isContact = True

                if isContact:
                    if isFirstContact:
                        print("reached region {}".format(target_idx+1))
                        snapshotTime = curr_time
                    isFirstContact = False
                    # print(curr_time - firstContactTime)
                    if curr_time - snapshotTime > 23:
                        isContact = False
                        isFirstContact = True
                        isRelease = True

                if isRelease and \
                        sum([abs(err) < 0.01 for err in trans_error]) == len(trans_error):
                    target_idx = target_idx + 1 if target_idx < num_regions else target_idx
                    isRelease = False
                    print("go to region {}".format(target_idx+1))

        pub_pos(curr_target)
        pub_contact_mode(isContact)
        rate.sleep()


if __name__ == "__main__":
    main()
