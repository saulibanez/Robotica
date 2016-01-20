//ejecutar kobuki gazebo: roslaunch kobuki_gazebo kobuki_empty_world.launch

//ir a la carpeta: cd catkin_ws
//compilar: catkin_make
//. ~/catkin_ws/devel/setup.bash
//Cntrl + R, poner devel

//nota: necesito usar loop_rate.sleep

#include <ros/ros.h>
//rostopic list
//rostopic info /mobile_base/events/bumper
#include <std_msgs/String.h>
#include <kobuki_msgs/BumperEvent.h>
//rostopic info /mobile_base/commands/velocity
#include <geometry_msgs/Twist.h>
#include "time.h"

float vx=0.2;
int estados=1;
int binario;
int girando;
int tiempo_aleatorio=rand()%5;
float giro_aleatorio=0;
const int Avance=1;
const int Chocar=0;
const int Gira=2;
ros::Time begin;
ros::Time later;
float a = 0.2;
float lineal, angular;

void chatterCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){

	ROS_INFO("bumper event [%d]", msg->bumper);
	binario=msg->state;
	girando=msg->bumper;

}
int main(int argc, char **argv)
   {
     ros::init(argc, argv, "chocaGira");
   
     ros::NodeHandle n;
     ros::Subscriber sub = n.subscribe("mobile_base/events/bumper", 1000, chatterCallback);
     ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
     ros::Rate loop_rate(10);
   
	//rosmsg show geometry_msgs/Twist
	geometry_msgs::Twist Coordenadas;	//me creo una variable de tipo geometry_msgs/Twist llamada Coordenadas
	Coordenadas.linear.y=0.0;
	Coordenadas.linear.z=0.0;
	Coordenadas.angular.x=0.0;
	Coordenadas.angular.y=0.0;
	Coordenadas.angular.z=0.0;

    int count = 0;
    while (ros::ok())
    {
	giro_aleatorio=(rand()%40-25)/10;
	if(giro_aleatorio<0.5){
		giro_aleatorio=giro_aleatorio+0.5;
			
	}

		switch (estados){
		case Chocar:
			if (binario==0)
				estados=Gira;
			break;
		case Avance:
			begin=ros::Time::now();
			if ((begin)>(later)){
				angular=0.0;
				vx=a;
				if (binario==1){
					vx=-a;
					estados=Chocar;
					later=begin + ros::Duration(2);
				}
			}
			break;
		case Gira:
			begin=ros::Time::now();
			ROS_INFO("gira [%d]", girando);
			if ((begin)>(later)){
				vx=0;
				switch(girando){
	    		case 2: //choque der
	    		ROS_INFO("choque der [%d]", girando);
	    			angular = giro_aleatorio;
	    			estados=Avance;
	    			break;
	    		case 0:  //choque izq
	    		ROS_INFO("choque izq [%d]", girando);
	    			angular = giro_aleatorio;
	    			estados=Avance;
	    			break;
	    		case 1:  //choque frontal
	    		ROS_INFO("choque frontal [%d]", girando);
	    			angular = giro_aleatorio;
	    			estados=Avance;
	    			break;
	        	}
	        	later=begin + ros::Duration(3);
	        }

		}

		Coordenadas.linear.x=vx;	//la variable linear.x se coloca aqui porque la vamos a ir modificando
		Coordenadas.angular.z=angular;
  
    	chatter_pub.publish(Coordenadas);
  
      ros::spinOnce();
  
      loop_rate.sleep();
      ++count;
    }
 	ros::spin();
    return 0;
  }
