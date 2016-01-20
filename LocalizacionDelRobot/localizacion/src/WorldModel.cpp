#include "WorldModel.h"



WorldModel::WorldModel() {

}

WorldModel::~WorldModel() {
	// TODO Auto-generated destructor stub
}


void
WorldModel::step()
{


	port1.frame_id_ = "/world";
	port1.setOrigin(tf::Vector3(3.0, 0, 0));
	port1.setRotation(tf::Quaternion(0,0 ,0 ,1));
	port1.stamp_ = ros::Time::now() + ros::Duration(2.0);
	port1.child_frame_id_ = "/port1";


	port2.frame_id_ = "/world";
	port2.setOrigin(tf::Vector3(-3.0, 0, 0));
	port2.setRotation(tf::Quaternion(0,0 ,0 ,1));
	port2.stamp_ = ros::Time::now() + ros::Duration(2.0);
	port2.child_frame_id_ = "/port2";


	lm4.frame_id_ = "/world";
	lm4.setOrigin(tf::Vector3(1.5, -2.15, 0));
	lm4.setRotation(tf::Quaternion(0,0 ,0 ,1));
	lm4.stamp_ = ros::Time::now() + ros::Duration(2.0);
	lm4.child_frame_id_ = "/lm4";


	lm3.frame_id_ = "/world";
	lm3.setOrigin(tf::Vector3(-1.5, -2.15, 0));
	lm3.setRotation(tf::Quaternion(0,0 ,0 ,1));
	lm3.stamp_ = ros::Time::now() + ros::Duration(2.0);
	lm3.child_frame_id_ = "/lm3";


	lm2.frame_id_ = "/world";
	lm2.setOrigin(tf::Vector3(-1.5, 2.15, 0));
	lm2.setRotation(tf::Quaternion(0,0 ,0 ,1));
	lm2.stamp_ = ros::Time::now() + ros::Duration(2.0);
	lm2.child_frame_id_ = "/lm2";

	lm1.frame_id_ = "/world";
	lm1.setOrigin(tf::Vector3(1.5, 2.15, 0));
	lm1.setRotation(tf::Quaternion(0,0 ,0 ,1));
	lm1.stamp_ = ros::Time::now() + ros::Duration(2.0);
	lm1.child_frame_id_ = "/lm1";

	bin.frame_id_ = "/world";
	bin.setOrigin(tf::Vector3(COORXBIN, COORYBIN, 0));
	bin.setRotation(tf::Quaternion(0,0 ,0 ,1));
	bin.stamp_ = ros::Time::now() + ros::Duration(2.0);
	bin.child_frame_id_ = "/bin";

	try
	{
		tfB.sendTransform(port1);
		tfB.sendTransform(port2);
		tfB.sendTransform(lm1);
		tfB.sendTransform(lm2);
		tfB.sendTransform(lm3);
		tfB.sendTransform(lm4);
		tfB.sendTransform(bin);
	}catch(tf::TransformException & ex)
	{
		ROS_WARN("%s",ex.what());
	}

}
