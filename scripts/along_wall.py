#!/usr/bin/python

import rospy
from math import cos, pi
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def lidar_callback(scan_msg):

    num_45deg_ray = int(round( (-pi/4 - scan_msg.angle_min) / scan_msg.angle_increment ))
    error = need_dist - scan_msg.ranges[num_45deg_ray]

    velocity = Twist()
    velocity.linear.x = forw_vel
    velocity.angular.z = kp * error
    vel_pub.publish(velocity)


if __name__=="__main__":

    rospy.init_node("along_wall_node")

    forw_vel = rospy.get_param("~forward_velocity", 1.0)
    kp = rospy.get_param("~controller_coefficient", 1.0)
    need_dist = rospy.get_param("~needed_distance", 2.0)
    need_dist /= cos(pi/4)

    lidar_sub = rospy.Subscriber("base_scan", LaserScan, lidar_callback)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    rospy.spin()
