#ifndef MCLRobotica_H_
#define MCLRobotica_H_

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "exps_sound/PoseWithProb.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include <boost/math/distributions.hpp>
#include <boost/math/distributions/normal.hpp>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <time.h>
#include <random>
typedef struct
{
	geometry_msgs::Pose coord;
	float p;
}Particle;


class MCLRobotica {
public:
	MCLRobotica();
	virtual ~MCLRobotica();

	void correct();
	void publishInfo();
	void predict();
	geometry_msgs::PoseWithCovarianceStamped getPose();

	void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
	//int getStates() {return NUMPARTICLES;};
private:

	static constexpr float field_width = 6.00;
	static constexpr float field_height = 5.25;
	static const int NUMPARTICLES = 100;
	static constexpr float percent_gauss = 1;
	static const int RESET_COUNT = 20;
	static constexpr float RESET_TH = 5.0;

	int creset;


	void resetParticles();
	void updatePos();
	void reseed();
	void normalize();

	void resetOdom();

	Particle particles[NUMPARTICLES];
	geometry_msgs::PoseWithCovariance pose;
	int seq;
	std::default_random_engine generator;


	void printParticles();


	void updateObservation2(std::string obs, std::string real);
	void publishOrientations();
	void publishPose();

	float getProbPos(float ideal, float obs, float desv);
	float getProbRot(float ideal, float obs, float desv);

	double normalizePi(double data);

	ros::NodeHandle n;

	ros::Publisher part_pub;
	ros::Publisher pose_pub;
	ros::Publisher lost_pub, notlost_pub;
	ros::Subscriber odom_sub;

	ros::Publisher resetodom_pub;
	tf::TransformListener tfL;
	tf::TransformBroadcaster tfB;

	nav_msgs::Odometry odom;
};

#endif /* MCLRobotica_H_ */
