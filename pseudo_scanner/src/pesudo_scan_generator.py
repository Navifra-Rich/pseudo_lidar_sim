#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

pose_robot  = PoseStamped()
pose_obstacle = PoseStamped()

def gen_scan():
    scan = LaserScan()
    scan.header.frame_id = "base_link"
    scan.header.stamp = rospy.Time.now()

    return

def obs_odom_callback(msg):
    global odom
    odom=msg
    return

def odom_callback(msg):
    map.pose = msg.pose
    map.getYawFromQuat()
    return

def main():

    rospy.init_node('map_generator', anonymous=True)
    sub_robot = rospy.Subscriber("/odom", Odometry, odom_callback)
    sub_obstacle = rospy.Subscriber("/obstacle_odom", Odometry, obs_odom_callback)


    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        continue
        # print(map.map)
        # print(map.pose)
        # print("HJI")
        # rate.sleep()

if __name__ == '__main__':
    main()
