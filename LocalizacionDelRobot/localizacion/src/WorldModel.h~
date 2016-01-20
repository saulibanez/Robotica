/*
 * WorldModel.h
 *
 *  Created on: 07/12/2014
 *      Author: paco
 */

#ifndef WORLDMODEL_H_
#define WORLDMODEL_H_

#include <string>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <iostream>
#include <list>
#include <algorithm>

class WorldModel {
public:
	WorldModel();
	virtual ~WorldModel();

	void step();

private:
	ros::NodeHandle n;


	tf::TransformBroadcaster tfB;
	
	tf::StampedTransform port1, port2, lm1, lm2, lm3, lm4, bin;

	static constexpr float COORXBIN = -3.0;
	static constexpr float COORYBIN = -2.15;
};

#endif /* WORLDMODEL_H_ */
