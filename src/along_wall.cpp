#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#define ROUND(X) ((X - std::floor(X) >= 0.5) ? (std::ceil(X)) : (std::floor(X)))

ros::Publisher vel_pub;
double kp, need_dist, forw_vel;

void lidar_callback(const sensor_msgs::LaserScan &scan_msg){

	int num_45deg_ray = ROUND( (-M_PI/4 - scan_msg.angle_min) / scan_msg.angle_increment );
	double error = need_dist - scan_msg.ranges[num_45deg_ray];

	geometry_msgs::Twist velocity;
	velocity.linear.x = forw_vel;
	velocity.angular.z = kp * error;
	vel_pub.publish(velocity);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "along_wall_node");

	ros::NodeHandle nh;
	ros::param::param<double>("~forward_velocity", forw_vel, 1.0);
	ros::param::param<double>("~controller_coefficient", kp, 1.0);
	ros::param::param<double>("~needed_distance", need_dist, 2.0);
	need_dist /= std::cos(M_PI/4);

	ros::Subscriber lidar_sub = nh.subscribe("base_scan", 1000, &lidar_callback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	ros::spin();
	return 0;
}
