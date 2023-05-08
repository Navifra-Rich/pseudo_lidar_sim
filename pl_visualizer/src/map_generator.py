#!/usr/bin/env python3

import os

import rospy
import numpy as np
import cv2

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
import getch

obs_odom = Odometry()
odom_pub = rospy.Publisher('/obstacle_moved', Odometry, queue_size=10)

obs_odom.header.frame_id="map"
obs_odom.child_frame_id="obstacle"


def obs_pose_init_callback(msg):
    global obs_odom
    obs_odom=msg
    return

def png_reader():
    map_image = cv2.imread('/home/hgnaseel/catkin_ws/src/pseudo_lidar_sim/pl_visualizer/data/map.png', cv2.IMREAD_GRAYSCALE)
    
    # 이미지 데이터를 0과 255 사이의 값으로 정규화
    map_image = cv2.normalize(map_image, None, 0, 255, cv2.NORM_MINMAX)

    # cv::Mat을 numpy array로 변환
    map_data = np.array(map_image, dtype=np.uint8)

     # map_metadata 생성
    map_info = OccupancyGrid()
    map_info.info.resolution = 0.05  # 셀 하나당 거리 (미터)
    map_info.info.width = map_data.shape[1]  # 맵의 가로 길이
    map_info.info.height = map_data.shape[0]  # 맵의 세로 길이
    map_info.info.origin.position.x = -1.0 * map_info.info.width * map_info.info.resolution / 2.0
    map_info.info.origin.position.y = -1.0 * map_info.info.height * map_info.info.resolution / 2.0
    map_info.info.origin.position.z = 0.0
    map_info.info.origin.orientation.w = 1.0

    # OccupancyGrid 메시지 생성
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = 'map'
    map_msg.header.stamp = rospy.Time.now()
    map_msg.info = map_info.info

    # 맵 데이터 채우기
    map_msg.data = np.zeros(map_data.shape[0] * map_data.shape[1], dtype=np.int8)
    for i in range(map_data.shape[0]):
        for j in range(map_data.shape[1]):
            if map_data[i, j] < 128:
                map_msg.data[i * map_data.shape[1] + j] = 100
            else:
                map_msg.data[i * map_data.shape[1] + j] = 0

    # map 토픽으로 메시지 발행
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    pub.publish(map_msg)




def keyboard_input():
    print("HERE")
    key = getch.getch()
    if key == 'w' :
        print("Up arrow is pressed")
        obs_odom.pose.pose.position.x +=0.1

    elif key == 's' :
        print("Down arrow is pressed")
        obs_odom.pose.pose.position.x -=0.1

    elif key == 'a' :
        obs_odom.pose.pose.position.y +=0.1
        print("Left arrow is pressed")

    elif key == 'd' :
        obs_odom.pose.pose.position.y -=0.1
        print("Right arrow is pressed")

    elif key == 'q' :
        obs_odom.pose.pose.position.y +=0.1
        print("Turn left")

    elif key == 'e' :
        obs_odom.pose.pose.position.y -=0.1
        print("Turn left")

    elif key == 'c' :
        rospy.signal_shutdown("CCC")
        return
    print(key)
    os.system('clear')

    obs_odom.header.stamp = rospy.Time.now()
    odom_pub.publish(obs_odom)




def odom_callback(msg):
    map.pose = msg.pose
    map.getYawFromQuat()
    return

def main():

    rospy.init_node('map_generator', anonymous=True)

    sub_obstacle = rospy.Subscriber("/obstacle_init", Odometry, obs_pose_init_callback)

    rate = rospy.Rate(100) # 10hz
    for i in range(10):
        png_reader()

    while not rospy.is_shutdown():
        keyboard_input()



if __name__ == '__main__':
    main()
