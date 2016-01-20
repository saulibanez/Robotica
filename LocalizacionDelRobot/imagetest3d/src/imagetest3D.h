#ifndef Imagetest3D
#define Imagetest3D
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types_conversion.h>
#include "lista.cpp"
#include <pcl_ros/transforms.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

// #include "lista.h"
// #include <list>
	typedef enum Colorines{YELLOW, MAGENTA, BLUE, ORANGE, LIGHT_ORANGE,RED, NUM_COLORINES}Colorines;

	typedef struct tagobject{
		float mediaX, mediaY, mediaZ;
		float xMax, yMax, zMax;
		float xMin, yMin, zMin;
		int num_pixel;
	}ColorObject;


class tresD{
	public:
		tresD();
		~tresD();
		void update(int numColor, float x, float y, float z);
		void imageCb(const ros::MessageEvent<sensor_msgs::PointCloud2 const>& msg);
		void seleccionar();
		void es_baliza();
		void es_pelota();
		void borrar();
	private:


	ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;

	int hupper, hlower;
	int supper, slower;
	int vupper, vlower;

	float hupper_yellow, hlower_yellow, supper_yellow, slower_yellow, vupper_yellow, vlower_yellow;
	float hupper_magenta, hlower_magenta, supper_magenta, slower_magenta, vupper_magenta, vlower_magenta;
	float hupper_blue, hlower_blue, supper_blue, slower_blue, vupper_blue, vlower_blue;
	float hupper_orange, hlower_orange, supper_orange, slower_orange, vupper_orange, vlower_orange;
	float hupper_light_orange, hlower_light_orange, supper_light_orange, slower_light_orange, vupper_light_orange, vlower_light_orange;
	float hupper_red, hlower_red, supper_red, slower_red, vupper_red, vlower_red;

	//ColorObject objects[NUM_COLORINES];

	// int num_pixel_yellow;
	// int num_pixel_magenta;
	// int num_pixel_blue;
	// int num_pixel_orange;
	// int num_pixel_ligth_orange;
	// int num_pixel_red;
	bool entrado;
	float nsupper, nslower;
	float nvupper, nvlower;
	tf::TransformListener tfL;
	tf::TransformBroadcaster tfB;
	tf::Transform tfs;


	int baliza[6];
	std::list<ColorObject> objects[NUM_COLORINES];
	ColorObject value;
	bool actualizar;
};
#endif