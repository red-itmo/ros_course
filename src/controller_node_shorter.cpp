#include <cmath>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>

#define KF 1.0
#define KR 2.0

class ControllerNode{
    ros::NodeHandle nh;
    ros::Subscriber goal_sub;
    ros::Subscriber pose_sub;
    ros::Publisher vel_pub;

    geometry_msgs::Point goal;

public:
    ControllerNode();

    void run();

    void goal_callback(const geometry_msgs::Point &msg);
    void pose_callback(const turtlesim::Pose &cur_pose);
};


ControllerNode::ControllerNode(){
    goal_sub = nh.subscribe("/turtle_goal", 1000, &ControllerNode::goal_callback, this);
    pose_sub = nh.subscribe("/turtle1/pose", 1000, &ControllerNode::pose_callback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    goal.x = 5.54;
    goal.y = 5.54;
}


void ControllerNode::goal_callback(const geometry_msgs::Point &msg){
    goal = msg;
}


void ControllerNode::pose_callback(const turtlesim::Pose &cur_pose){

    double x = goal.x - cur_pose.x;
    double y = goal.y - cur_pose.y;
    double dist_to_goal = std::sqrt(x*x + y*y);

    double error_angle;
    error_angle = angles::shortest_angular_distance(angles::normalize_angle(cur_pose.theta), std::atan2(y, x));

    geometry_msgs::Twist velocity;
    if(std::abs(dist_to_goal) > 0.05){
        velocity.linear.x = KF * dist_to_goal * std::cos(error_angle);
        velocity.angular.z = KR * error_angle;
        vel_pub.publish(velocity);
    }
}


void ControllerNode::run(){
    ros::spin();
}


int main(int argc, char **argv){
    ros::init(argc, argv, "controller_node_cpp");
    ControllerNode node;
    node.run();
    return 0;
}
