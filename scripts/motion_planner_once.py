#! /usr/bin/env python3
'''
go to target once
'''
import numpy as np
import rospy
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool


# ---------------------constant transformations-----------------------------

# transformation from base to eef
# (data recorded at home pose, for debug purpose)
T_O_ee = np.array([[-0.02406, -0.9997, -0.0001, 0.0],
                   [-0.999, 0.02405, -0.0275, 0.0],
                   [0.02751, -0.00055, -0.9996, 0.0],
                   [0.26308, 0.025773, 0.2755, 1.0]]).transpose()

# transformation from camera to target
T_cam_tar = None

# transformation from custom eef to camera [m]
T_ee_cam = np.array([[1.000, 0.0, 0.0, -0.0175],
                     [0.0, 0.9239, -0.3827, -0.0886180],
                     [0.0, 0.3827, 0.9239, -0.3233572],
                     [0.0, 0.0, 0.0, 1.0]])
# --------------------------------------------------------------------------


def cam_tar_callback(msg):
    cam_tar = list(msg.data)
    # transformation from camera to target
    global T_cam_tar
    T_cam_tar = np.array([[cam_tar[0], cam_tar[3], cam_tar[6], cam_tar[9]],
                          [cam_tar[1], cam_tar[4], cam_tar[7], cam_tar[10]],
                          [cam_tar[2], cam_tar[5], cam_tar[8], cam_tar[11]],
                          [0.0, 0.0, 0.0, 1.0]])
    # print(T_cam_tar)


def ee_callback(data):
    EE_pos = data.O_T_EE  # inv 4x4 matrix
    global T_O_ee
    T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12],
                       EE_pos[12:16]]).transpose()
    # ee_sub.unregister()


def convert2base():
    if T_O_ee is not None and T_cam_tar is not None:
        T_O_cam = np.matmul(T_O_ee, T_ee_cam)
        T_O_tar = np.matmul(T_O_cam, T_cam_tar)
    else:
        T_O_tar = None
    return T_O_tar


def pub_pos(T_O_tar):
    tar_msg = Float64MultiArray()
    tar_packed = np.transpose(
        np.array([T_O_tar[0], T_O_tar[1], T_O_tar[2]])).flatten()
    tar_msg.data = tar_packed
    if not rospy.is_shutdown():
        tar_pub.publish(tar_msg)


def pub_op(contactFlag=False):
    op_msg = Bool()
    op_msg.data = contactFlag
    if not rospy.is_shutdown():
        op_pub.publish(op_msg)


tar_sub = rospy.Subscriber(
    'cam_target', Float64MultiArray, cam_tar_callback)
ee_sub = rospy.Subscriber(
    "franka_state_controller/franka_states", FrankaState, ee_callback)
tar_pub = rospy.Publisher('franka_cmd_pos', Float64MultiArray, queue_size=1)
op_pub = rospy.Publisher('isContact', Bool, queue_size=1)


def main():
    isContact = False
    rospy.init_node('waypoints_generator', anonymous=True)
    dist_coeff = 0.075
    T_O_tar = convert2base()
    if T_O_tar is not None:
        Vz = T_O_tar[0:3, 2]
        P0 = T_O_tar[0:3, 3]
        # entry point
        Pz = np.subtract(P0, [dist_coeff*Vzi for Vzi in Vz])
        T_O_tar[0:3, 3] = Pz
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():

            # print("base target: \n", T_O_tar)

            T_error = np.subtract(T_O_tar, T_O_ee)
            trans_error = T_error[0:3, 3]
            rot_error = T_error[0:3, 0:3].flatten()

            isReachedTrans = True if sum(
                [abs(err) < 0.002 for err in trans_error]) == len(trans_error) else False
            isReachedRot = True if sum(
                [abs(err) < 0.35 for err in rot_error]) == len(rot_error) else False
            if isReachedRot and isReachedTrans:
                isContact = True

            print("trans error: \n", trans_error)
            print("rot error: \n", rot_error)
            print("trans arrival: ", isReachedTrans,
                  "\nrot arrival: ", isReachedRot)

            pub_pos(T_O_tar)
            pub_op(isContact)
            rate.sleep()


if __name__ == '__main__':
    main()
