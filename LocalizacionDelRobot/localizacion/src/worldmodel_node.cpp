#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>

#include "WorldModel.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("worldmodel"));
	ros::NodeHandle n;

	WorldModel worldmodel;

	ros::Rate loop_rate(1);
	int count = 0;

	while (ros::ok())
	{
		worldmodel.step();

    	ros::spinOnce();

    	loop_rate.sleep();
    	++count;
	}
	return 0;
}
