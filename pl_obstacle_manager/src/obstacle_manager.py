#!/usr/bin/env python3

import rospy
import numpy as np

import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

import math
from nav_msgs.msg import Odometry
import os
import struct
import random


# ------------------------------ Param ------------------------------------
SIZEX = 1.9
SIZEY = 0.6
LASER_RANGE = 100.0
class Object:
    def __init__(self):
        self.odom = Odometry()
        self.odom.header.frame_id="map"
        self.odom.pose.pose.orientation.w=1
        return


init_pub = rospy.Publisher("/obstacle_init", Odometry)
obs_odom_pub = rospy.Publisher("/obstacle_odom", Odometry)
rob_odom_pub = rospy.Publisher("/robot_odom", Odometry)
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

obstacle = Object()
robot = Object()

ray_x = []
ray_y = []
def getRays():
    # 거리 값 배열 생성
    for i in range(0, 629):
        ray_x.append(LASER_RANGE*math.cos(i/100))  # 거리 값은 10으로 설정
        ray_y.append(LASER_RANGE*math.sin(i/100))  # 거리 값은 10으로 설정

    return ray_x, ray_y

def obs_odom_callback(msg):
    obstacle.odom=msg
    return

def odom2marker(obs_odom):
    marker = Marker()
    marker.header = obs_odom.header
    marker.type = Marker.CUBE
    marker.pose = obs_odom.pose.pose
    marker.scale.x = SIZEX
    marker.scale.y = SIZEY
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

def pub_points(points):
    # PointCloud 메시지 생성
    msg = PointCloud2()

    # Header 설정
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    # PointField 추가
    msg.fields.append(PointField('x', 0, PointField.FLOAT32, 1))
    msg.fields.append(PointField('y', 4, PointField.FLOAT32, 1))
    msg.fields.append(PointField('z', 8, PointField.FLOAT32, 1))
    msg.fields.append(PointField('rgb', 12, PointField.FLOAT32, 1))

    # 필드 정보 설정
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * len(points)
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = True

    for i in range(len(points)):
        # print
        point = [float(i) for i in [points[i][0],points[i][1], 3, 0]]
        # print(point)
        # 바이트 변환하여 PointCloud 메시지에 추가
        msg.data += struct.pack('ffff', *point)
    pub = rospy.Publisher("pseudo_scan", PointCloud2)
    pub.publish(msg)
    return msg

def line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
    if denom == 0: # parallel
        return None
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
    if ua < 0 or ua > 1: # out of range
        return None
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom
    if ub < 0 or ub > 1: # out of range
        return None
    x = x1 + ua * (x2-x1)
    y = y1 + ua * (y2-y1)
    return (x,y)

def collision_check(robot, corner):
    scan_array = []
    for i in range(len(ray_x)):
        min_dist = 99999
        scanned_points = None
        # print("HERE")
        pos = line_intersection(robot.x, robot.y, ray_x[i], ray_y[i], corner[0,0], corner[1,0], corner[0,1], corner[1,1])
        if pos is not None:
            dist = (pos[0]-robot.x)*(pos[0]-robot.x)+(pos[1]-robot.y)*(pos[1]-robot.y)
            if dist<min_dist:
                min_dist=dist
                scanned_points=pos
        pos = line_intersection(robot.x, robot.y, ray_x[i], ray_y[i], corner[0,1], corner[1,1], corner[0,3], corner[1,3])
        if pos is not None:
            dist = (pos[0]-robot.x)*(pos[0]-robot.x)+(pos[1]-robot.y)*(pos[1]-robot.y)
            if dist<min_dist:
                min_dist=dist
                scanned_points=pos
        pos = line_intersection(robot.x, robot.y, ray_x[i], ray_y[i], corner[0,0], corner[1,0], corner[0,2], corner[1,2])
        if pos is not None:
            dist = (pos[0]-robot.x)*(pos[0]-robot.x)+(pos[1]-robot.y)*(pos[1]-robot.y)
            if dist<min_dist:
                min_dist=dist
                scanned_points=pos
        pos = line_intersection(robot.x, robot.y, ray_x[i], ray_y[i], corner[0,2], corner[1,2], corner[0,3], corner[1,3])
        if pos is not None:
            dist = (pos[0]-robot.x)*(pos[0]-robot.x)+(pos[1]-robot.y)*(pos[1]-robot.y)
            if dist<min_dist:
                min_dist=dist
                scanned_points=pos

        if scanned_points is not None:
            scan_array.append(scanned_points)

    # print(scan_array)
    return scan_array

def getCornerPoints(obstacle_quat, robot_quat):
    corner = np.zeros((2,4))
    corner[0,0] = corner[0,2] = -SIZEX/2
    corner[0,1] = corner[0,3] = SIZEX/2

    corner[1,0] = corner[1,1] = SIZEY/2
    corner[1,2] = corner[1,3] = -SIZEY/2

    obs_yaw = euler_from_quaternion([obstacle_quat.x, obstacle_quat.y, obstacle_quat.z, obstacle_quat.w])[2]
    robot_yaw = euler_from_quaternion([robot_quat.x, robot_quat.y, robot_quat.z, robot_quat.w])[2]
    yaw = obs_yaw 

    print("----------------------------")
    print(f"obs_yaw = {obs_yaw}") 
    print(f"robot_yaw = {robot_yaw}") 
    print(corner)

    # print(math.cos(yaw))

    new_corner_x = corner[0]*math.cos(yaw) - corner[1]*math.sin(yaw) 
    new_corner_y = corner[0]*math.sin(yaw) + corner[1]*math.cos(yaw) 
    corner[0] = new_corner_x
    corner[1] = new_corner_y
    print(corner)

    return corner

def odom_moved_callback(msg):
    global obstacle
    obstacle.odom.pose.pose.position = msg.pose.pose.position
    return

def gen_callback(msg):
    print("----------------------------     IN GEN CVALLBACK")
    global obstacle
    obstacle.odom.pose.pose = msg.pose
    print(msg.pose)
    print(obstacle.odom.pose.pose)
    # obstacle.odom.pose.pose.position = msg.pose.position
    obstacle.odom.header.stamp = rospy.Time.now()
    obstacle.odom.header.frame_id = "map"
    init_pub.publish(obstacle.odom)
    print("----------------------------")
    return

def robot_odom_callback(msg):
    robot.odom=msg
    robot.odom.header.frame_id = "map"
    return

def main():
    global obstacle

    rospy.init_node('obstacle_manager', anonymous=True)

    sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, gen_callback)
    robot_odom_sub = rospy.Subscriber("/odom", Odometry, robot_odom_callback)
    sub_obstacle = rospy.Subscriber("/obstacle_init", Odometry, obs_odom_callback)
    odom_moved_sub = rospy.Subscriber('/obstacle_moved', Odometry, odom_moved_callback)
    getRays()
    br = tf.TransformBroadcaster()


    listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10hz

    robot.odom.pose.pose.position.x = 0
    robot.odom.pose.pose.position.y = 0
    while not rospy.is_shutdown():
        os.system('clear')

        obstacle.odom.header.stamp = rospy.Time.now()
        robot.odom.header.stamp=rospy.Time.now()

        obs_odom_pub.publish(obstacle.odom)
        rob_odom_pub.publish(robot.odom)

        rel_pose = PoseStamped()
        rel_pose.pose.position.x = obstacle.odom.pose.pose.position.x - robot.odom.pose.pose.position.x 
        rel_pose.pose.position.y = obstacle.odom.pose.pose.position.y - robot.odom.pose.pose.position.y 



        obs_corner = getCornerPoints(obstacle.odom.pose.pose.orientation, robot.odom.pose.pose.orientation)
        obs_corner[0] += obstacle.odom.pose.pose.position.x
        obs_corner[1] += obstacle.odom.pose.pose.position.y

        print(f"Obs   pose {obstacle.odom.pose.pose}")
        print(f"robot pose {robot.odom.pose.pose}")
        print(f"OBS CORNER {obs_corner}")
        scan_array = collision_check(robot.odom.pose.pose.position, obs_corner)

        pub_points(scan_array)
        marker_pub.publish(odom2marker(obstacle.odom))



if __name__ == '__main__':
    main()

