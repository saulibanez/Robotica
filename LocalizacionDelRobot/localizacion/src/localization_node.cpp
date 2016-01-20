#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string>
#include "MCLRobotica.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("localization"));
	ros::NodeHandle n;
	tf::TransformListener tfL;
	tf::StampedTransform W2BL;
	tf::TransformBroadcaster tfB;
	ros::Publisher pose_pub;
	pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_pos", 1000);
	ros::Rate loop_rate(10);
	geometry_msgs::PoseWithCovarianceStamped robotPose;
	int count = 0;
	MCLRobotica mcl;

	while (ros::ok())
	{	
		mcl.predict(); //HAY QUE PROBAR!?
		robotPose = mcl.getPose();

		W2BL.frame_id_ = "/world";
		W2BL.child_frame_id_ = "/base_link";
		W2BL.stamp_ = ros::Time::now() + ros::Duration(1.0);


		W2BL.setOrigin(tf::Vector3(robotPose.pose.pose.position.x, robotPose.pose.pose.position.y, 0.0));
		W2BL.setRotation(tf::Quaternion(robotPose.pose.pose.orientation.x, robotPose.pose.pose.orientation.y,
			robotPose.pose.pose.orientation.z, robotPose.pose.pose.orientation.w));

		try{
			tfB.sendTransform(W2BL);
		}catch(tf::TransformException & ex){
			ROS_WARN("%s",ex.what());
		}
		
		mcl.correct();
		mcl.publishInfo();		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
