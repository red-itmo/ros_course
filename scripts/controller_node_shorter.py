#!/usr/bin/python

import rospy
import math
import threading
import angles
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

#Controller's parameters
kf = 1.0 #f - forward
kr = 2.0 #r - rotation

goal = Point()
goal.x = 5.54
goal.y = 5.54

lock = threading.Lock()

def goal_callback(msg):
    global goal
    lock.acquire()
    goal = msg
    lock.release()

def pose_callback(cur_pose):
    lock.acquire()
    x = goal.x - cur_pose.x
    y = goal.y - cur_pose.y
    lock.release()
    dist_to_goal = math.sqrt(x**2 + y**2)
    error_angle = angles.shortest_angular_distance(angles.normalize_angle(cur_pose.theta), math.atan2(y, x))

    velocity = Twist()
    if abs(dist_to_goal) > 0.05:
        velocity.linear.x = kf * dist_to_goal * math.cos(error_angle)
        velocity.angular.z = kr * error_angle
        vel_pub.publish(velocity)


if __name__=="__main__":
    rospy.init_node('controller_node')
    goal_sub = rospy.Subscriber('/turtle_goal', Point, goal_callback)
    pose_sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.spin()
