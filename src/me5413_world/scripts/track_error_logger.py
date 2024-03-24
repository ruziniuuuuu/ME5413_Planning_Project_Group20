#!/usr/bin/env python

import rospy
import rospkg
import os
from std_msgs.msg import Float32

position_error = None
heading_error = None
speed_error = None


def position_callback(msg):
    global position_error
    position_error = msg.data


def heading_callback(msg):
    global heading_error
    heading_error = msg.data


def speed_callback(msg):
    global speed_error
    speed_error = msg.data


def track_error_logger(bag_file_path):
    rospy.init_node('track_error_logger')
    rospy.Subscriber('/me5413_world/planning/rms_position_error', Float32, position_callback)
    rospy.Subscriber('/me5413_world/planning/rms_heading_error', Float32, heading_callback)
    rospy.Subscriber('/me5413_world/planning/rms_speed_error', Float32, speed_callback)

    rate = rospy.Rate(10)  # 10hz
    with open(bag_file_path, 'w') as f:
        while not rospy.is_shutdown():
            data = "{},{},{}\n".format(position_error, heading_error, speed_error)
            f.write(data)
            rate.sleep()


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('me5413_world')
    data_path = os.path.join(package_path, 'result')
    absolute_data_path = os.path.abspath(data_path)
    bag_file_path = os.path.join(absolute_data_path, 'errors.csv')
    # bag_file_path = os.path.join(absolute_data_path, 'errors_pid_Stanley.csv')
    # bag_file_path = os.path.join(absolute_data_path, 'errors_pid_purePursuit.csv')

    try:
        track_error_logger(bag_file_path)
    except rospy.ROSInterruptException:
        pass