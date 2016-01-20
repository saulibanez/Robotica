#include "ros/ros.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <kobuki_msgs/Sound.h>
#include "usarPid.h"
geometry_msgs::PoseStamped pose;

geometry_msgs::PoseStamped lastPose;
static const int SEARCH = 0;
static const int GOTOBALL = 1;
static const int GOTOBIN = 2;
static const int GIRANDO = 3;
static const int AVANZAR_ESPARTANOS = 4;
static const int VE_HACIA_ATRAS = 5;
static ros::Time ETA;
static ros::Time ETA2;
static ros::Time Tiempo_Hacia_Atras;

static float diffpose2;
static float angle2goal2;


static const int NUMBOLAS = 3;
static const float VELOCIDAD_ANGULAR =0.3; //medida en radianes/segundo me la he inventado

					//calculad tiempo que tarda en dar una vuelta entera yendo a 0.3 en angular.z
					//2*PI/ese_tiempo os da la velocidad angular

					//Tarda alrededor de 23 segundos en dar una vuelta completa
					// Tiempo = Distancia / Velocidad
static const float VELOCIDAD_LINEAL = 0.25; //medida en m/s me la he inventado
					//calculad el tiempo que tarda en avanzar un metro yendo a 0.3 en linear.x
					//1/ese_tiempo os da la velocidad lineal

					//Tarda casi 4 segundos en recorrer un metro
bool lost = false;


inline double normalizePi(double data)
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


void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pose = *msg;
}

void lostCB(const std_msgs::Empty::ConstPtr& msg){
	std::cout<<"LLEGA LOST"<<std::endl;
	lost = true;
}

void notlostCB(const std_msgs::Empty::ConstPtr& msg){
	std::cout<<"LLEGA NOTLOST"<<std::endl;
	lost = false;
}

bool isPrefix(std::string const& s1, std::string const&s2)
{
	return s1.compare(s2.substr(0, s1.length()))==0;
}

bool isBlacklisted(std::string frame, std::string blacklist[NUMBOLAS], int bolas)
{
	for(int i = 0; i < bolas; i++)
		if(blacklist[i].compare(frame) == 0)
			return true;
	return false;
}

float getDistanceTo(std::string frame)
{
	tf::StampedTransform BL2B;
	tf::TransformListener tfL;
	try{
		tfL.lookupTransform("base_link", frame,
				ros::Time::now() - ros::Duration(1.0), BL2B);
		return sqrt(
			BL2B.getOrigin().y() * BL2B.getOrigin().y()
					+ BL2B.getOrigin().x()
							* BL2B.getOrigin().x());
	}catch(tf::TransformException & ex) {
		ROS_WARN("%s", ex.what());
		return 9.99f;
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, std::string("behavior"));

	kobuki_msgs::Sound sound;

	tf::TransformListener tfL;

	ros::NodeHandle n;

	Pid pid;
	ros::Subscriber robotposesub = n.subscribe("/MCLRobotica_pos", 1000, &poseCB);
	ros::Publisher  cmdpub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
	ros::Publisher sonido = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound",1);
	ros::Subscriber lost_sub = n.subscribe("/MCLRobotica_lost", 1000, &lostCB);
	ros::Subscriber notlost_sub = n.subscribe("/MCLRobotica_notlost", 1000, &notlostCB);
  // ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);


	ros::Rate loop_rate(10);

	
	int count = 0;
	int state = SEARCH;
	std::string target;
	std::string blacklist[NUMBOLAS];
	int bolas = 0; 
	lastPose.pose.position.z = -1.0;

	geometry_msgs::Twist cmd;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = 0.0;
	cmd.linear.x = 0.0;
	float angle2goal;

	while (ros::ok())
	{
		if(bolas == 3)
			break;
		switch(state){
		case SEARCH: 
		{
			target = "A";
			std::vector<std::string> frameList;
			tfL.getFrameStrings(frameList);
			std::vector<std::string>::iterator it;
			float dmin = 99.9;
			for (it = frameList.begin(); it != frameList.end(); ++it) {
				std::string frame = *it;
				tf::StampedTransform BL2B;

				try {
					tfL.lookupTransform("base_link", frame,
						ros::Time::now(), BL2B);

					if (isPrefix("ball_", frame) && !isBlacklisted(frame, blacklist, bolas)) {
					int d = getDistanceTo(frame);
					if(dmin > d){
						dmin = d;
						target = frame;
					}
					state = GOTOBALL;
				}
				} catch (tf::TransformException & ex) {
					;//ROS_WARN("%s", ex.what());
				}
				
			}
			if(target.compare("A")==0)
				std::cout<<"NADA"<<std::endl;
			else
				std::cout<<"TARGET = "<<target<<std::endl;
			cmd.linear.x = 0.2;
			if(cmd.angular.z <= 0.4)
				cmd.angular.z = cmd.angular.z + 0.01;

			break;
		}
		
		case GOTOBALL:
		{
			tf::StampedTransform BL2B;
			try{
				tfL.lookupTransform("base_link", target,
						ros::Time::now() - ros::Duration(1.0), BL2B);
			}catch(tf::TransformException & ex) {
				ROS_WARN("LA HE PERDIDO: %s", ex.what());
				state = SEARCH;
				cmd.angular.z = 0.0;
				break;
			}
			float ro;
				
			ro = sqrt((BL2B.getOrigin().x())*(BL2B.getOrigin().x()) + (BL2B.getOrigin().y())*(BL2B.getOrigin().y())); 
			double roll, pitch, yaw;
			float v,w, usoPid;
			Pid velDeGiro;
			float theta;
			std::cout<<"pelota "<<target<<" ro = "<<ro<<std::endl;				

			if(ro < 0.7){	
				w = v = 0.0;
				sound.value=kobuki_msgs::Sound::CLEANINGSTART;
	         sonido.publish(sound); //AQUI SE REPRODUCE UN SONIDO
				state = GOTOBIN;
				cmd.linear.x = v;
				cmd.angular.z = w;
			}else{
				theta = normalizePi(atan2(BL2B.getOrigin().y(), BL2B.getOrigin().x()));
				std::cerr<<"theta: "<<fabs(theta)<<std::endl;
				usoPid=velDeGiro.OperarMiPid(theta);

				cmd.angular.z = usoPid;
				if(fabs(theta) > 0.1){
					v = 0.1;
					cmd.linear.x = v;
				}else{
					v = 0.2;//AQUI HABRIA QUE HACER UN VFF SENCILLO QUE SOLO TENGA EN CUENTA LA DISTANCIA HASTA LA PELOTA
					cmd.linear.x = v;
				}
			}
			break;
		}
		case GOTOBIN: 
		{
			if(lost){
				std::cout<<"LAST POSE: "<<lastPose.pose.position.z<<std::endl;
				if(lastPose.pose.position.z == -1.0){
					if(count % 200 < 160){
						std::cout<<"giro para encontrarme"<<std::endl;
						cmd.angular.z = 0.2;
						cmd.linear.x = 0.0;
					}else{
						std::cout<<"Avanzo para encontrarme"<<std::endl;
						cmd.linear.x = 0.1;
						cmd.angular.z = 0.0;
					}
				}else{

					tf::StampedTransform W2BIn;
					try{
						tfL.lookupTransform("world", "bin",
							ros::Time::now() - ros::Duration(1.0), W2BIn);

					}catch(tf::TransformException & ex) {
						ROS_WARN("%s", ex.what());
						break;
					}
					double roll, pitch, yaw;
					tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
					tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
					angle2goal2 = normalizePi(atan2(W2BIn.getOrigin().y() - lastPose.pose.position.y,
													 W2BIn.getOrigin().x() -lastPose.pose.position.x) - yaw);

					ETA = ros::Time::now() + ros::Duration(fabs(angle2goal2*23)/(2*M_PI));

					state = GIRANDO;
					diffpose2 = sqrt((lastPose.pose.position.x-W2BIn.getOrigin().x())*(lastPose.pose.position.x-W2BIn.getOrigin().x()) 
											  	+ (lastPose.pose.position.y-W2BIn.getOrigin().y())*(lastPose.pose.position.y-W2BIn.getOrigin().y()));
					
					
				}
				break;
			}else{

				tf::StampedTransform W2BIn;
				try{
					tfL.lookupTransform("world", "bin",
						ros::Time::now() - ros::Duration(1.0), W2BIn);

				}catch(tf::TransformException & ex) {
					ROS_WARN("%s", ex.what());
					break;
				}
				float diffpose;
				lastPose = pose;
				diffpose = sqrt((pose.pose.position.x-W2BIn.getOrigin().x())*(pose.pose.position.x-W2BIn.getOrigin().x()) 
					  	+ (pose.pose.position.y-W2BIn.getOrigin().y())*(pose.pose.position.y-W2BIn.getOrigin().y())); 
				double roll, pitch, yaw;
				tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
				tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
				float v,w, usoPid;
				Pid velDeGiro;
				if(diffpose < 0.1){	
					//std::cerr<<"diffpose: "<<diffpose<<std::endl;			

					w = v = 0.0;
					sound.value=kobuki_msgs::Sound::CLEANINGSTART;
	           	sonido.publish(sound);
					//AQUI SE REPRODUCE UN SONIDO Y SE ELIMINA LA PELOTA
					blacklist[bolas] = target;
					bolas++;
					state = SEARCH;
					cmd.angular.z = 0.0;
					cmd.linear.x = 0.0;
					lastPose.pose.position.z = -1;
				}else{
					angle2goal = normalizePi(atan2(W2BIn.getOrigin().y() - pose.pose.position.y,
												 W2BIn.getOrigin().x() -pose.pose.position.x) - yaw);
					//std::cerr<<"angle2goal: "<<fabs(angle2goal)<<std::endl;	
					usoPid=velDeGiro.OperarMiPid(angle2goal);		
					cmd.angular.z = usoPid;
					if(fabs(angle2goal) > 0.2){
						v = 0.0;
						cmd.linear.x = v;
					}else{
						v = 0.3;//AQUI HABRIA QUE HACER UN VFF SENCILLO QUE SOLO TENGA EN CUENTA LA DISTANCIA HASTA BIn
						cmd.linear.x = v;
					}
				}
				break;	//si no funciona según lo esperado, este break lo tenía puesto antes encima del case GIRANDO, pero creo que aqui
							//hace mejor su función
			}
		}
		case GIRANDO:
		{
			cmd.angular.z = 0.0;
			cmd.angular.y = 0.0;
			cmd.angular.x = 0.0;
			cmd.linear.x = 0.0;
			cmd.linear.y = 0.0;
			cmd.linear.z = 0.0;
			if(ros::Time::now() < ETA){
				if(angle2goal2>=0){
					std::cout<<"entro a girar izquierda"<<std::endl;
					cmd.angular.z = 0.3;
				}else{
					std::cout<<"entro a girar derecha"<<std::endl;
					cmd.angular.z = -0.3;
				}

			}else{
        		state = AVANZAR_ESPARTANOS;
        		ETA2 = ros::Time::now() + ros::Duration(diffpose2/VELOCIDAD_LINEAL);
			}
			cmdpub.publish(cmd);
			break;
		}
		case AVANZAR_ESPARTANOS:
		{
			cmd.angular.z = 0.0;
			cmd.angular.y = 0.0;
			cmd.angular.x = 0.0;
			cmd.linear.x = 0.0;
			cmd.linear.y = 0.0;
			cmd.linear.z = 0.0;
			if(ros::Time::now() < ETA2){
				std::cout<<"entro a avanzar"<<std::endl;
				cmd.linear.x = 0.3;

			}else{
				sound.value=6;//kobuki_msgs::Sound::CLEANINGSTART;
        		sonido.publish(sound);
        		blacklist[bolas] = target;
				bolas++;
				state = VE_HACIA_ATRAS;
				//voy a retroceder el mismo tiempo y a la misma velocidad de lo que habia avanzado
				Tiempo_Hacia_Atras = ros::Time::now() + ros::Duration(diffpose2/VELOCIDAD_LINEAL);
			}
			
			//lastPose.pose.position.z = -1;
			cmdpub.publish(cmd);
			break;
		}
		case VE_HACIA_ATRAS:
		{
			cmd.angular.z = 0.0;
			cmd.angular.y = 0.0;
			cmd.angular.x = 0.0;
			cmd.linear.x = 0.0;
			cmd.linear.y = 0.0;
			cmd.linear.z = 0.0;
			if(ros::Time::now() < Tiempo_Hacia_Atras){
				std::cout<<"voy a dar marcha atras"<<std::endl;
				cmd.linear.x = -0.3;
			}else{
				state = SEARCH;
			}
			lastPose.pose.position.z = -1;
			cmdpub.publish(cmd);
			break;
		}

		default:
			std::cout<<"WHAT THE FUCK"<<std::endl;
		}
		cmdpub.publish(cmd);		
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
