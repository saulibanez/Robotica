#include "MCLRobotica.h"


MCLRobotica::MCLRobotica() {
	std::cout<<"YIEH"<<std::endl;
	srand(time(0));
	generator.seed(time(NULL));

	resetParticles();
	updatePos();

	part_pub = n.advertise<geometry_msgs::PoseArray>("/MCLRobotica_particles", 1000);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/MCLRobotica_pos", 1000);
	odom_sub = n.subscribe<nav_msgs::Odometry>("/robot1/odom", 10, &MCLRobotica::odomCB,
			this);
	lost_pub = n.advertise<std_msgs::Empty>("/MCLRobotica_lost", 1000);
	notlost_pub = n.advertise<std_msgs::Empty>("MCLRobotica_notlost", 1000);

	resetodom_pub = n.advertise<std_msgs::Empty>("/robot/commands/reset_odometry", 1);

	seq = 0;
	creset = 0;
	resetOdom();

}


MCLRobotica::~MCLRobotica() {

}

void MCLRobotica::resetParticles() {

	for (int i = 0; i < NUMPARTICLES; i++) {
		float x, y, t;
		particles[i].p = 1.0 / ((float) NUMPARTICLES);

		do {
			x = ((float) rand() / (float) RAND_MAX) * field_height
					- (field_height / 2.0);
			y = ((float) rand() / (float) RAND_MAX) * field_width
					- (field_width / 2.0);
		} while ((x < -(field_height / 2.0)) || (x > (field_height / 2.0))
				|| (y < -(field_width / 2.0)) || (y > (field_height / 2.0)));

		t = normalizePi(((float) rand() / (float) RAND_MAX) * 2.0 * M_PI);

		particles[i].coord.position.x = x;
		particles[i].coord.position.y = y;
		particles[i].coord.position.z = 0.0;

		tf::Quaternion q;
		q.setEuler(0.0, 0.0, t);

		particles[i].coord.orientation.x = q.x();
		particles[i].coord.orientation.y = q.y();
		particles[i].coord.orientation.z = q.z();
		particles[i].coord.orientation.w = q.w();

	}
}



void MCLRobotica::updatePos() {


	//std::cerr<<"U<";
	float x, sx2;
	float y, sy2;
	float xa, sxa2;
	float ya, sya2;
	float t, st2;
	double roll, pitch, yaw;

	x = 0.0;
	y = 0.0;
	t = 0.0;
	xa = ya = 0.0;


	for (int i = 0; i < NUMPARTICLES; i++) {
		float cx, cy;
		float cax, cay;

		cx = particles[i].coord.position.x;
		cy = particles[i].coord.position.y;


		tf::Quaternion q(particles[i].coord.orientation.x,
				particles[i].coord.orientation.y,
				particles[i].coord.orientation.z,
				particles[i].coord.orientation.w);

		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		cax = cos(yaw);
		cay = sin(yaw);

		//sdt::cerr<<"\t\t("<<cax<<","<<cay<<"): "<< particles[i].p<<std::endl;

		x = x + cx * particles[i].p;
		y = y + cy * particles[i].p;

		xa = xa + cax * particles[i].p;
		ya = ya + cay * particles[i].p;


		//sdt::cerr<<"\t\t---->("<<xa<<","<<ya<<")"<<std::endl;
	}
	//sdt::cerr<<"\t\t---->("<<xa<<","<<ya<<")"<<std::endl;


	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = 0.0;


	t = atan2(ya, xa);

	sx2 = 0.0;
	sy2 = 0.0;
	sxa2 = 0.0;
	sya2 = 0.0;
	st2 = 0.0;

	for (int i = 0; i < NUMPARTICLES; i++) {
		float cx, cy;
		float cax, cay;

		cx = particles[i].coord.position.x;
		cy = particles[i].coord.position.y;

		tf::Quaternion q(particles[i].coord.orientation.x,
				particles[i].coord.orientation.y,
				particles[i].coord.orientation.z,
				particles[i].coord.orientation.w);

		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		cax = cos(yaw);
		cay = sin(yaw);

		sx2 = sx2 + particles[i].p * ((cx - x) * (cx - x));
		sy2 = sy2 + particles[i].p * ((cy - y) * (cy - y));

		sxa2 = sxa2 + particles[i].p * ((cax - xa) * (cax - xa));
		sya2 = sya2 + particles[i].p * ((cay - ya) * (cay - ya));

		float taux= normalizePi(yaw - t);

		st2 = st2 + particles[i].p * taux * taux;

	}


	tf::Quaternion q;

	q.setEuler(0.0, 0.0, t);

	pose.pose.orientation.x = q.x();
	pose.pose.orientation.y = q.y();
	pose.pose.orientation.z = q.z();
	pose.pose.orientation.w = q.w();


	for (int i = 0; i < 36; i++)
		pose.covariance[i] = 0.0;

	pose.covariance[0] = sx2; //X*X
	pose.covariance[1 * 6 + 1] = sy2; //Y*Y
	pose.covariance[5 * 6 + 5] = st2; //rZ*rZ
	//std::cerr<<">"<<std::endl;

}

void MCLRobotica::resetOdom() {
	odom.pose.pose.position.x = 0;
	odom.pose.pose.position.y = 0;
	odom.pose.pose.position.z = 0;

	tf::Quaternion q;
	q.setEuler(0.0, 0.0, 0.0);
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();

}

void MCLRobotica::odomCB(const nav_msgs::Odometry::ConstPtr& msg) {
	
	odom = *msg;


}


void MCLRobotica::predict() {

	tf::Transform desp;

	desp.setOrigin(
			tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y,
					odom.pose.pose.position.z));
	std::cout<<"x "<<odom.pose.pose.position.x<<"y "<<odom.pose.pose.position.y<<std::endl;
	desp.setRotation(
			tf::Quaternion(odom.pose.pose.orientation.x,
					odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
					odom.pose.pose.orientation.w));

	if (desp.getRotation().getX() != desp.getRotation().getX())
		return;

	for (int i = 0; i < NUMPARTICLES; i++) {
		tf::Transform part, parttf;

		part.setOrigin(
				tf::Vector3(particles[i].coord.position.x,
						particles[i].coord.position.y,
						particles[i].coord.position.z));
		part.setRotation(
				tf::Quaternion(particles[i].coord.orientation.x,
						particles[i].coord.orientation.y,
						particles[i].coord.orientation.z,
						particles[i].coord.orientation.w));

		parttf = part * desp;

		particles[i].coord.position.x = parttf.getOrigin().getX();
		particles[i].coord.position.y = parttf.getOrigin().getY();
		particles[i].coord.position.z = parttf.getOrigin().getZ();
		particles[i].coord.orientation.x = parttf.getRotation().getX();
		particles[i].coord.orientation.y = parttf.getRotation().getY();
		particles[i].coord.orientation.z = parttf.getRotation().getZ();
		particles[i].coord.orientation.w = parttf.getRotation().getW();

	}

	std_msgs::Empty rmsg;
	resetodom_pub.publish(rmsg);
}

double MCLRobotica::normalizePi(double data)
{
  if (data < M_PI && data >= -M_PI) return data;
  double ndata = data - ((int )(data / (M_PI*2.0)))*(M_PI*2.0);
  while (ndata >= M_PI)
  {
    ndata -= (M_PI*2.0);
  }
  while (ndata < -M_PI)
  {
    ndata += (M_PI*2.0);
  }
  return ndata;
}


void MCLRobotica::publishOrientations() {
	geometry_msgs::PoseArray paray;
	static int seq = 0;

	paray.header.frame_id = "world";
	paray.header.stamp = ros::Time::now();
	paray.header.seq = seq++;

	for (int i = 0; i < NUMPARTICLES; i++)
		paray.poses.push_back(particles[i].coord);

	part_pub.publish(paray);
}

void MCLRobotica::publishPose() {
	geometry_msgs::PoseStamped paray;
	static int seq = 0;

	paray.header.frame_id = "world";
	paray.header.stamp = ros::Time::now();
	paray.header.seq = seq++;

	paray.pose = pose.pose;

	pose_pub.publish(paray);
}


bool isPrefix(std::string const& s1, std::string const&s2)
{
	return s1.compare(s2.substr(0, s1.length()))==0;

}

void MCLRobotica::publishInfo() {

	publishPose();
	publishOrientations();
}

void MCLRobotica::correct() {
	std::vector<std::string> frameList;

	tfL.getFrameStrings(frameList);

	std::vector<std::string>::iterator it;

	int numObjetos = 0;

	for (it = frameList.begin(); it != frameList.end(); ++it) {

		std::string frame = *it;

		if (isPrefix("perceived_", frame)) {

			std::string parent;

			tf::StampedTransform BL2F;

			parent = frame.substr(std::string("perceived_").length(),
					std::string::npos);
			try {
				tfL.lookupTransform("base_link", frame,
					ros::Time::now(), BL2F);

				updateObservation2(frame, parent);
				numObjetos++;
			} catch (tf::TransformException & ex) {
				;//ROS_WARN("%s", ex.what());
			}
		}

	}
float sump = 0;
	for (int i = 0; i < NUMPARTICLES; i++){
		sump = sump + particles[i].p;
		//std::cout<<"particula "<<i<<" prob: "<<particles[i].p<<std::endl;
	}

	sump = sump/(float)NUMPARTICLES;

	std_msgs::Empty rmsg;
	if(numObjetos < 2 && sump<RESET_TH){
	std::cout<<"perdido"<<std::endl;
		creset++;
		lost_pub.publish(rmsg);
	}else{
		creset = 0;
		notlost_pub.publish(rmsg);
			std::cout<<"localizado"<<std::endl;

	}
	if(creset>RESET_COUNT){
		creset = 0;
		std::cout<<"RESET"<<std::endl;
		//resetParticles();
	}
	std::cout<<"prob: "<<sump<< " creset: "<<creset<<std::endl;

	
	normalize();

	updatePos();
	reseed();
}






void MCLRobotica::updateObservation2(std::string obs, std::string real) {

	//Calcular la posición del robot en coordenadas de world, dada una observación

	tf::StampedTransform R2O, W2R;

	try {
		tfL.lookupTransform("base_link", obs,
				ros::Time::now(), R2O);
		tfL.lookupTransform("world", "base_link",
				ros::Time::now() - ros::Duration(0.3), W2R);

	} catch (tf::TransformException & ex) {
		ROS_WARN("%s", ex.what());
	}

	//Comprobamos que está en el campo de visión (57º)

	tf::Matrix3x3 w2r(W2R.getRotation());
	double roll, pitch, yaw;
	w2r.getRPY(roll, pitch, yaw);

	float angle2obs = atan2(R2O.getOrigin().y(), R2O.getOrigin().x());
	float dista2obs = sqrt(
			R2O.getOrigin().y() * R2O.getOrigin().y()
					+ R2O.getOrigin().x()
							* R2O.getOrigin().x());

	if (fabs(angle2obs) > (57.0 * M_PI / 180.0))
		return;

	//std::cerr<<"O"<<std::endl;
	tf::StampedTransform W2L;
	try {
		tfL.lookupTransform("world", real,
				ros::Time::now() - ros::Duration(0.2), W2L);
	} catch (tf::TransformException & ex) {
		ROS_WARN("%s", ex.what());
	}
	

	static int c = 0;
	double mayor = 0.1;

	for (int i = 0; i < NUMPARTICLES; i++) {
		tf::Transform W2H, H2L;

		W2H.setOrigin(
				tf::Vector3(particles[i].coord.position.x,
						particles[i].coord.position.y,
						particles[i].coord.position.z));
		W2H.setRotation(
				tf::Quaternion(particles[i].coord.orientation.x,
						particles[i].coord.orientation.y,
						particles[i].coord.orientation.z,
						particles[i].coord.orientation.w));

		tf::Matrix3x3 m(W2H.getRotation());
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);


		H2L = W2H.inverse() * W2L;

		tf::Matrix3x3 w2r(H2L.getRotation());
		//double roll, pitch, yaw;
		w2r.getRPY(roll, pitch, yaw);

		float x, y;

		x = H2L.getOrigin().x();// * cos(-yaw) - H2L.getOrigin().y() * sin(-yaw); //Porque es la vista desde la baliza al robot
		y = H2L.getOrigin().y();//; * sin(-yaw) + H2L.getOrigin().y() * cos(-yaw);

		float angle2ideal = normalizePi(atan2(y, x)/* + M_PI*/);
		float dista2ideal = sqrt(x * x + y * y);
/*		 std::cout<<"Angulo a ideal: "<<angle2ideal << " distancia ideal: " << dista2ideal<<std::endl;
		 std::cout<<"Angulo observacion: " << angle2obs << " distancia obs " << dista2obs <<std::endl;*/

		float desvDist = 0.2; //20 cms
		float desvAngl = 0.1; //0.1

		float probdist = getProbPos(dista2ideal, dista2obs, desvDist);
		float probrota = getProbRot(angle2ideal, angle2obs, desvAngl);

		if(particles[i].p == -1){
			particles[i].p = probdist * probrota;
		}else{
			particles[i].p = particles[i].p * probdist * probrota;
		}

	//	std::cout<<i<<": probdist "<<probdist<<" probrota "<<probrota<<" ptotal: "<<particles[i].p<<std::endl;

		if (particles[i].p < 0.0000001)
			particles[i].p = 0.0000001;

	}
}
geometry_msgs::PoseWithCovarianceStamped MCLRobotica::getPose() {
	geometry_msgs::PoseWithCovarianceStamped ret;

	ret.header.seq = seq++;
	ret.header.frame_id = "world";
	ret.header.stamp = ros::Time::now();

	ret.pose = pose;

	return ret;
}

void MCLRobotica::normalize() {

	//std::cerr<<"N<";

	float sum = 0.0;
	float factor;

	for (int i = 0; i < NUMPARTICLES; i++)
		sum = sum + particles[i].p;

	factor = 1.0 / sum;
	for (int i = 0; i < NUMPARTICLES; i++)
		particles[i].p = particles[i].p * factor;

	//std::cerr<<">"<<std::endl;;
}


float MCLRobotica::getProbPos(float ideal, float obs, float desv) {
	float dist;
	dist = fabs(ideal - obs);
//	std::cout<<"dist = "<<dist<<std::endl;
	boost::math::normal_distribution<> myNormal(0.0, desv);
	//std::cout<<"pdf: "<<pdf(myNormal, 0.0)<<std::endl;

	return pdf(myNormal, dist);

}

float MCLRobotica::getProbRot(float ideal, float obs, float desv) {

	double diff;
	diff = normalizePi(ideal - obs);
//std::cout<<"diff = "<<diff<<std::endl;
	boost::math::normal_distribution<> myNormal(0.0, desv);

	return pdf(myNormal, diff);

}

void MCLRobotica::reseed() {

	//std::cerr<<"R<";
	float sx2, sy2, st2;

	//printParticles();
	//std::cerr<<"["<<pose.covariance[0]<<" ->";

	pose.covariance[0] = pose.covariance[0] + 0.01;
	pose.covariance[1 * 6 + 1] = pose.covariance[1 * 6 + 1] + 0.01;
	pose.covariance[5 * 6 + 5] = pose.covariance[5 * 6 + 5] + 0.01;

	sx2 = pose.covariance[0]; //X*X
	sy2 = pose.covariance[1 * 6 + 1]; //Y*Y
	st2 = pose.covariance[5 * 6 + 5]; //rZ*rZ

	//sx2 = 0.1;
	//sy2 = 0.1;
	//st2 = 0.1;

	//std::cerr<<"["<<sx2<<"]"<<std::endl;
		double roll, pitch, yaw;
	tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
			pose.pose.orientation.z, pose.pose.orientation.w);

	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	//std::cout<<"desviaciones: "<<sx2<<", "<<sy2<<", "<<st2<<std::endl;

	std::normal_distribution<float> normalX(0.0, sqrt(sx2));
	std::normal_distribution<float> normalY(0.0, sqrt(sy2));

	std::normal_distribution<float> normalT(0.0, sqrt(st2));
	/*
	 boost::math::normal_distribution<> normalX(0.0, sqrt(sx2));
	 boost::math::normal_distribution<> normalY(0.0, sqrt(sy2));
	 boost::math::normal_distribution<> normalT(0.0, sqrt(st2));
	 */

	//std::cerr<<"Normal con sx2 = ["<<sqrt(sx2)<<"]"<<std::endl;

				

	for (int i = 0; i < NUMPARTICLES*percent_gauss; i++) {

		float x, y;
		do {
			x = pose.pose.position.x + normalX(generator);
			y = pose.pose.position.y + normalY(generator);

		} while ((x < -(field_height / 2.0)) || (x > (field_height / 2.0))
				|| (y < -(field_width / 2.0)) || (y > (field_height / 2.0)));

		particles[i].coord.position.x = x;
		particles[i].coord.position.y = y;
		particles[i].coord.position.z = 0;

		tf::Quaternion q;
		q.setEuler(0.0, 0.0, yaw + normalizePi((double) normalT(generator)));

		particles[i].coord.orientation.x = q.x();
		particles[i].coord.orientation.y = q.y();
		particles[i].coord.orientation.z = q.z();
		particles[i].coord.orientation.w = q.w();
		particles[i].p = -1;
	}
	//std::cerr<<">"<<std::endl;

	for (int i = NUMPARTICLES*percent_gauss; i < NUMPARTICLES; i++) {

		float x, y;
		do {
			x = ((float) rand() / (float) RAND_MAX) * field_height
					- (field_height / 2.0);
			y = ((float) rand() / (float) RAND_MAX) * field_width
					- (field_width / 2.0);

		} while ((x < -(field_height / 2.0)) || (x > (field_height / 2.0))
				|| (y < -(field_width / 2.0)) || (y > (field_height / 2.0)));

		particles[i].coord.position.x = x;
		particles[i].coord.position.y = y;
		particles[i].coord.position.z = 0;

		float t = normalizePi(((float) rand() / (float) RAND_MAX) * 2.0 * M_PI);

		tf::Quaternion q;
		q.setEuler(0.0, 0.0, t);


		particles[i].coord.orientation.x = q.x();
		particles[i].coord.orientation.y = q.y();
		particles[i].coord.orientation.z = q.z();
		particles[i].coord.orientation.w = q.w();
		particles[i].p = -1;
	}
}

void MCLRobotica::printParticles() {
	for (int i = 0; i < NUMPARTICLES; i++)
		std::cerr << "[" << i << "] (" << particles[i].coord.position.x << ", "
				<< particles[i].coord.position.y << ") " << particles[i].p
				<< std::endl;
}