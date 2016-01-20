/*
 * 
 * 
 * Autor: Francisco Martín Rico (fmrico@gmail.com)
 * Fecha: 11/02/2014
 *  
 * Programa de prueba de filtrado de imagen en HSV para asignatura de robótica en la URJC
 * 
 * La imagen proviene de la imagen publicada en /camera/rgb/image_raw ; cambiar si es otra
 * 
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int hupper, hlower; 
  
public:
  ImageConverter(): it_(nh_)
  {
    
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    cvNamedWindow( "Imagen Fuente");
    cvNamedWindow( "Imagen filtrada");
   
	// TrackBar
	cvCreateTrackbar("Hue Upper","Imagen filtrada",&hupper,360,NULL);
	cvCreateTrackbar("Hue Lower","Imagen filtrada",&hlower,360,NULL);
	//cvCreateButton("Save",ImageConverter::callbackButton,NULL,CV_PUSH_BUTTON,0);
 
  }

 ~ImageConverter()
 {
	cv::destroyWindow("Imagen Fuente");
    cv::destroyWindow("Imagen filtrada");
 }
 static void callbackButton(int state,void* userdata)
 {
	printf("yeah");
}
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	//fprintf(stderr, "h\n");
    cv_bridge::CvImagePtr cv_ptr, cv_imageout;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_imageout = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


	cv::Mat hsv;  
    cv::cvtColor (cv_ptr->image, hsv, CV_RGB2HSV);
    
   	int height = hsv.rows;
	int width = hsv.cols;
	int step = hsv.step;
	int channels = 3;

	
	for(int i = 0; i <height; i++ )
          for(int j = 0; j <width; j++ ) 
          {
			int posdata = i*step+j*channels; //Only H channel
			
			if(!((hsv.data[posdata] >= hlower) && (hsv.data[posdata] <= hupper)))
			{
				cv_imageout->image.data[posdata] = 0;
				cv_imageout->image.data[posdata+1] = 0;
				cv_imageout->image.data[posdata+2] = 0;
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
  ImageConverter ic;
  ros::spin();
  return 0;
}
