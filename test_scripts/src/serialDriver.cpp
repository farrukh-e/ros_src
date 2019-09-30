#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>


void left_encoder_callback(std_msgs::Int16 &msg){
	ROS_INFO("Left encoder speed: [%s]", msg.data);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "odometry_publisher");
	ros::init(argc, argv, "left_ticks_sub");
	ros::init(argc, argv, "right_ticks_sub");

	ros::NodeHandle nh;
	ros::Subscriber left_ticks_sub = nh.subscribe("/left_ticks", 1000, left_encoder_callback);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(5.0);  //publish Rate of 5 Hz
	while(nh.ok()){

		ros::spinOnce();
		current_time = ros::Time::now();

	}


}






