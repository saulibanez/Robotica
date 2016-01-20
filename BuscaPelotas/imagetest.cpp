/*
 * Autor: Saúl Ibáñez Cerro
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Sound.h>
#include "time.h"
#include "usarPid.h"

const int Avance=0;
const int Gira_Avanza=1;
const int Gira=2;
const int Pito=3;
ros::Time begin;
ros::Time later;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  int hUpper, hLower;
  int sUpper, sLower;
  int vUpper, vLower;

public:
  ImageConverter(): it_(nh_)
  {
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    cvNamedWindow( "Imagen Fuente");
    cvNamedWindow( "Imagen filtrada");
    
    hUpper=131;
    sUpper=255;
    vUpper =255;
    hLower=96;//110;
    sLower=125;
    vLower=139;
  	// TrackBar
  	cvCreateTrackbar("Hue Upper","Imagen filtrada",&hUpper,255,NULL);
  	cvCreateTrackbar("Hue Lower","Imagen filtrada",&hLower,255,NULL);

    cvCreateTrackbar("Saturacion Upper","Imagen filtrada",&sUpper,255,NULL);
    cvCreateTrackbar("Saturacion Lower","Imagen filtrada",&sLower,255,NULL);

    cvCreateTrackbar("Value Upper","Imagen filtrada",&vUpper,255,NULL);
    cvCreateTrackbar("Value Lower","Imagen filtrada",&vLower,255,NULL);
 
  }

 ~ImageConverter(){
	cv::destroyWindow("Imagen Fuente");
  cv::destroyWindow("Imagen filtrada");
 }


 static void callbackButton(int state,void* userdata)
 {
	printf("yeah");
  }

void imageCb(const sensor_msgs::ImageConstPtr& msg){

  cv_bridge::CvImagePtr cv_ptr, cv_imageout;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_imageout = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

	cv::Mat hsv;  
  cv::cvtColor (cv_ptr->image, hsv, CV_RGB2HSV);
    
  int height = hsv.rows;
	int width = hsv.cols;
	int step = hsv.step;
	int channels = 3;

	for(int i = 0; i <height; i++ ){
    for(int j = 0; j <width; j++ ){
			int posdata = i*step+j*channels; //Only H channel--> solo utiliza la h
                                        //posdata +1 considero la s
                                        //posdata +2 considero la v
			
			if(!((hsv.data[posdata] >= hLower) && (hsv.data[posdata] <= hUpper))||
        !((hsv.data[posdata+1] >= sLower) && (hsv.data[posdata+1] <= sUpper))||
        !((hsv.data[posdata+2] >= vLower) && (hsv.data[posdata+2] <= vUpper))){
  				cv_imageout->image.data[posdata] = 0;
  				cv_imageout->image.data[posdata+1] = 0;
  				cv_imageout->image.data[posdata+2] = 0;
      }
		}
  }

    // Update GUI Window
    cv::imshow("Imagen Fuente", cv_ptr->image);
    cv::imshow("Imagen filtrada", cv_imageout->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_imageout->toImageMsg());

  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  //ros::NodeHandle n;
  
  ImageConverter ic;
  ros::spin();
  return 0;
}
