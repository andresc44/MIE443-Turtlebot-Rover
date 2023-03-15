#include <robot_pose.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/PoseStamped.h> // Added

geometry_msgs::PoseStamped GlobalRobotPose; // Added

RobotPose::RobotPose(float x, float y, float phi) {
	this->x = x;
	this->y = y;
	this->phi = phi;
}

void RobotPose::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
	GlobalRobotPose.header = msg.header; //Added
	GlobalRobotPose.pose = msg.pose.pose; //Added
	phi = tf::getYaw(msg.pose.pose.orientation);
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
}
