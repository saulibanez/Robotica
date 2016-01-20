/*
 * Autor: Saúl Ibáñez Cerro
 */
#ifndef BuscoPelotas
#define BuscoPelotas
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Sound.h>
#include "usarPid.h"
#include "time.h"



class Pelotas{
	public:
		Pelotas();
		void OperarBuscarPelotas(const sensor_msgs::ImageConstPtr& msg);
	private:
		float x,y,vx;
		int contadorPixeles;
		int hUpper, hLower;
		int sUpper, sLower;
		int vUpper, vLower;
		int estados;
		int height;
		int width;
		int step;
		int channels;
		 //creo las variables para usar el PID
		float error;
		Pid velDeGiro;
		Pid TotalMedia;
		float usoPid;
		static const int Avance=0;
		static const int Gira_Avanza=1;
		static const int Gira=2;
		static const int Pito=3;
		ros::Time begin;
		ros::Time later;
		ros::NodeHandle nh_;
		kobuki_msgs::Sound sound;
  		geometry_msgs::Twist Coordenadas;
  		ros::Subscriber sub;
  		ros::Publisher chatter_pub;
		ros::Publisher sonido;
		bool VerPelotaRoja, VerPelotaNarPeq, VerPelotaNarGra;
		bool exitRoja,exitNarPeq,exitNarGra;

		int arr [10];
		int ValueH;
		int ValueS;
        int ValueV;
        int contador;
        int TotalFiltrado;
        int SumarCeldasArray;
        float Total;
};
#endif