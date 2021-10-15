#! /usr/bin/env python3
'''
record force
'''
import rospy
import csv
import numpy as np
from geometry_msgs.msg import WrenchStamped


def force_callback(msg):
    global Fx, Fy, Fz
    Fx = msg.wrench.force.x
    Fy = msg.wrench.force.y
    Fz = msg.wrench.force.z


rospy.Subscriber('/franka_state_controller/F_ext',
                 WrenchStamped, force_callback)


def main():
    rospy.init_node('force_data_logger', anonymous=True)
    path2file = '/home/xihan/catkin_ws/src/robotic_ultrasound/scripts/force_data_log.csv'
    file_out = open(path2file, 'w')
    writer = csv.writer(file_out)
    rate = rospy.Rate(50)
    print('start recording.')

    while not rospy.is_shutdown():
        data = [Fx, Fy, Fz]
        writer.writerow(data)
        rate.sleep()
    print('\nend recording.')
    file_out.close()


if __name__ == "__main__":
    main()
