/*
 * Autor: Saúl Ibáñez Cerro
 */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "buscaPelotas.h"
Pelotas::Pelotas(){
  estados=0;

  //cuento los pixeles para saber el centro de la pelota
  x=0.0;
  y=0.0;
  contadorPixeles=0;
  VerPelotaRoja=false;
  VerPelotaNarPeq=false;
  VerPelotaNarGra=false;
  exitRoja=false;
  exitNarPeq=false;
  exitNarGra=false;

  vx=0.2;
  hUpper=131;
  sUpper=255;
  vUpper =255;
  hLower=96;//110;
  sLower=120;
  vLower=139;
  usoPid=0;

  ValueH=0;
  ValueS=0;
  ValueV=0;
  contador=0;
  TotalFiltrado=0;
  SumarCeldasArray=0;
  Total=0.0;

  for (int i = 0; i < 10; i++)
  {
    arr[i]=0;
  }

  begin=ros::Time::now();
  later=begin + ros::Duration(2);
  sub = nh_.subscribe("/image_converter/output_video", 1, &Pelotas::OperarBuscarPelotas, this);
  chatter_pub = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1); //Publicación del topic "cmd_vel"
  sonido = nh_.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound",1);

}
void Pelotas::OperarBuscarPelotas(const sensor_msgs::ImageConstPtr& msg){

  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat hsv;  
  cv::cvtColor (cv_ptr->image, hsv, CV_RGB2HSV);
  height = hsv.rows;
  width = hsv.cols;
  step = hsv.step;
  channels = 3;
  contadorPixeles=0;
	x=0.0;
  y=0.0;
  vx=0.2;

  /*
  * Recorremos la imagen en horizontal y vertical para poder calcular los pixeles
  */
  for(int i = 0; i < height; i++ ){
    for(int j = 0; j < width; j++ ){
			int posdata = i*step+j*channels; //Only H channel--> solo utiliza la h
                                        //posdata +1 considero la s
                                        //posdata +2 considero la v
			
			if(((hsv.data[posdata] >= hLower) && (hsv.data[posdata] <= hUpper))&&
        ((hsv.data[posdata+1] >= sLower) && (hsv.data[posdata+1] <= sUpper))&&
        ((hsv.data[posdata+2] >= vLower) && (hsv.data[posdata+2] <= vUpper))){
          ValueH= hsv.data[posdata];
          ValueS= hsv.data[posdata+1];
          ValueV=hsv.data[posdata+2];

  				contadorPixeles++;
          x=x+(float)i;
          y=y+(float)j;
  		
  		}

      /*if(((hsv.data[posdata] >= 96) && (hsv.data[posdata] <= 118))&&
        ((hsv.data[posdata+1] >= 120) && (hsv.data[posdata+1] <= 255))&&
        ((hsv.data[posdata+2] >= 139) && (hsv.data[posdata+2] <= 255))&&
        (VerPelotaNarGra==false)&&(contadorPixeles>60)){
          VerPelotaNarGra=true;
          ROS_INFO("NarGra");

      }else if(((hsv.data[posdata] >= 116) && (hsv.data[posdata] <= 133))&&
      ((hsv.data[posdata+1] >= 143) && (hsv.data[posdata+1] <= 255))&&
      ((hsv.data[posdata+2] >= 220) && (hsv.data[posdata+2] <= 255))&&
      (VerPelotaNarPeq==false)&&((contadorPixeles>1900)&&contadorPixeles<2050)
      &&(VerPelotaNarGra=true)){
          VerPelotaNarPeq=true;
           ROS_INFO("NarPeq");

      }else if(((hsv.data[posdata] >= 120) && (hsv.data[posdata] <= 140))&&
      ((hsv.data[posdata+1] >= 143) && (hsv.data[posdata+1] <= 218))&&
      ((hsv.data[posdata+2] >= 100) && (hsv.data[posdata+2] <= 189))&&
      (VerPelotaRoja==false)&&((contadorPixeles>1800)&&contadorPixeles<1950)
      &&(VerPelotaNarGra=true)&&(VerPelotaNarPeq=true)){
          VerPelotaRoja=true;
          ROS_INFO("Roja");
      }*/
    }
  }

  TotalFiltrado=ValueH+ValueS+ValueV;

  //averiguamos el centro de la pelota
  x=x/(float)contadorPixeles;
  y=y/(float)contadorPixeles;

  switch (estados){
    case Avance:
      if (contadorPixeles>60){
        if((y>200)&&(y<400)){
//ROS_INFO("Avance////////////////////////////////////////: [%f]", vx);
          Total=TotalMedia.OperarMiMedia(TotalFiltrado);

          Coordenadas.linear.x=vx;
          //ROS_INFO("total media+++++++++++++: [%f]", Total);
          if (x>=440){
            ROS_INFO("total media+++++++++++++: [%f]", Total);

            Total=TotalMedia.OperarMiMedia(TotalFiltrado);
            if ((Total>380)&&(Total<420)&&(VerPelotaNarGra==false)){
              ROS_INFO("Naranja grande");
              VerPelotaNarGra=true;
              sound.value=kobuki_msgs::Sound::CLEANINGSTART;
              sonido.publish(sound);
              estados=Gira;
              break;
            }else if ((Total>455)&&(Total<500)&&(VerPelotaNarPeq==false)){
              ROS_INFO("Naranja pequeña");

              VerPelotaNarPeq=true;
              sound.value=kobuki_msgs::Sound::CLEANINGSTART;
              sonido.publish(sound);
              estados=Gira;
              break;
            //}else if ((Total>410)&&(Total<448)&&(VerPelotaRoja==false)){
            }else if((VerPelotaNarPeq==true)&&(VerPelotaNarGra==true)&&(Total>410)&&(Total<448)&&(VerPelotaRoja==false)){
              ROS_INFO("Roja");

              VerPelotaRoja=true;
              sound.value=kobuki_msgs::Sound::CLEANINGSTART;
              sonido.publish(sound);
              estados=Gira;
              break;
            }
//ROS_INFO("PITPOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO: [%f]", x);

            /*if((VerPelotaNarGra==true)&&(exitNarGra==false)){
              ROS_INFO("pelota naranja grande vista: [%f]", x);

                exitNarGra=true;
                sound.value=kobuki_msgs::Sound::CLEANINGSTART;
                sonido.publish(sound);
                estados=Gira;
                break;
            }else if((VerPelotaNarPeq==true)&&(exitNarPeq==false)){
              ROS_INFO("pelota naranja pequeña vista: [%f]", x);
                exitNarPeq=true;
                sound.value=kobuki_msgs::Sound::CLEANINGSTART;
                sonido.publish(sound);
                estados=Gira;
                break;
            }else if((VerPelotaRoja==true)&&(exitRoja==false)){
              ROS_INFO("pelota roja vista: [%f]", x);
                exitRoja=true;
                sound.value=kobuki_msgs::Sound::CLEANINGSTART;
                sonido.publish(sound);
                estados=Gira;
                break;
            }*/
            
            //ROS_INFO("pelota vista: [%f]", Total);
            estados=Gira;
            break;
          }
        }else{
//ROS_INFO("girp av: [%f]", vx);

          estados=Gira_Avanza;
        }
      }else{
        Coordenadas.angular.z=0;
        Coordenadas.linear.x=0;
        estados=Gira;
      }
      break;
    case Gira_Avanza:
      if (contadorPixeles>60){
        if((y<=200)||(y>=400)){
//ROS_INFO("girp av y giro---------------------: [%f]", vx);
          /*
          * El error es en realidad la ecuacion de la recta, que luego 
          * usare para poder obtener el calculo del PID, implementado 
          * en usarPID.cpp
          */
          error = y/320 -1 ;//(y-320.0)/640.0;
          usoPid=velDeGiro.OperarMiPid(error);

          Coordenadas.linear.x=vx;
          Coordenadas.angular.z=(-1.0*usoPid);
        }else{
          estados=Avance;
        }
      }else{
        Coordenadas.angular.z=0;
        Coordenadas.linear.x=0;
        estados=Gira;
      }
      break;
    case Gira:
//ROS_INFO("giro+++++++++++++++++++++++++++++++++++++: [%f]", (-1.0*usoPid));
      if (contadorPixeles<=60){
        Coordenadas.angular.z=-0.5;
      }else{
        estados=Gira_Avanza;
      }
     break;


    }

    /*if ((VerPelotaRoja==true)&&(VerPelotaNarPeq==true)&&(VerPelotaNarGra==true)&&(exitRoja==false)){
      exitRoja=true;
      sound.value=6;
      sonido.publish(sound);
    }*/

    chatter_pub.publish(Coordenadas);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "buscaPelotas");
  Pelotas ball;
  ros::spin();    //Mantiene la suscripción al topic hasta que se reciba "Ctrl+C"
  return 0;
}