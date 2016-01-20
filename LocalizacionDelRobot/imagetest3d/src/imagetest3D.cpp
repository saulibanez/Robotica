/*
 *  
 * Programa de prueba de filtrado de imagen en HSV, se va a guardar los datos obtenidos en una lista enlazada
 * que se enviara por tf a dos programas distintos, uno localización, al que le pasaremos la ubicacion de las balizas y
 * las porterias para lograr que el robot se localice, y otro de comportamiento, al cual le mandaremos la posicion de 
 * las pelotas y el robot irá a por ellas.
 * 
 */

 // para la camara en 3d
 // roslaunch openni_launch openni.launch depth_registration:=true

 //para compilar más rapido:		catkin_make -j 4
 //para que cuando vaya a compilar entre tambien los launch:			rm -rf build/

 // roslaunch openni_launch openni.launch depth_registration:=true publish_tf:=false


#include "imagetest3D.h"


tresD::tresD(){

	hupper=340;
	hlower=272;
	supper=255;
	slower=108;
	vupper=205;
	vlower=55;
	image_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &tresD::imageCb, this);
	//image_sub_ = nh_.subscribe("/camera/depth/points", 1, &tresD::imageCb, this);
	//image_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &tresD::imageCb, this);
	image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_filtered", 1);

	nsupper=0.0;
	nslower=0.0;
	nvupper, nvlower=0.0; 

	hupper_magenta = 340.0;
	hlower_magenta = 272.0;
	supper_magenta = ((float)255)/255.0f;
	slower_magenta = ((float)108)/255.0f;
	vupper_magenta = ((float)205)/255.0f;
	vlower_magenta = ((float)55)/255.0f;

	nsupper = ((float)supper)/255.0f;
	nslower = ((float)slower)/255.0f;
	nvupper = ((float)vupper)/255.0f;
	nvlower = ((float)vlower)/255.0f;

	hupper_blue = 221.0;
	hlower_blue = 206.0;
	supper_blue = 255.0/255.0f;
	slower_blue = 195.0/255.0f;
	vupper_blue = 255.0/255.0f;
	vlower_blue = 120.0/255.0f;

	hupper_orange = 360.0;
	hlower_orange = 270.0;
	supper_orange = 255.0/255.0f;
	slower_orange = 254.0/255.0f;
	vupper_orange = 255.0/255.0f;
	vlower_orange = 184.0/255.0f;

	hupper_yellow = 74.0;
	hlower_yellow = 33.0;
	supper_yellow = 255.0/255.0f;
	slower_yellow = 139.0/255.0f;
	vupper_yellow = 255.0/255.0f;
	vlower_yellow = 80.0/255.0f;

	hupper_light_orange = 34.0;
	hlower_light_orange = 0.0;
	supper_light_orange = 248.0/255.0f;
	slower_light_orange = 161.0/255.0f;
	vupper_light_orange = 255.0/255.0f;
	vlower_light_orange = 132.0/255.0f;

	hupper_red = 360.0;
	hlower_red = 318.0;
	supper_red = 255.0/255.0f;
	slower_red = 189.0/255.0f;
	vupper_red = 255.0/255.0f;
	vlower_red = 32.0/255.0f;

	for (int i = 0; i < 6; i++){
		baliza[i]=0;
	}
}

tresD::~tresD(){
	// cv::destroyWindow("Imagen Fuente");
	// cv::destroyWindow("Imagen filtrada");
}

void tresD::update(int numColor, float x, float y, float z){
	actualizar = false;
	ColorObject obj;

	if(objects[numColor].size() == 0){
		//Si la lista de magenta está vacía.
		obj.mediaX = x;
		obj.mediaY = y;
		obj.mediaZ =  z;
		obj.xMax = x;
		obj.yMax = y;
		obj.zMax = z;
		obj.xMin = x;
		obj.yMin = y;
		obj.zMin = z;
		obj.num_pixel = 1;

		objects[numColor].push_back(obj);

		// cout <<x<<std::endl;
		// cout <<y<<std::endl;
		// cout <<z<<std::endl;
	
	}else{
		//Si la lista de magenta NO está vacía.
		entrado = false;
		std::list<ColorObject>::iterator itc;
		for(itc = objects[numColor].begin(); itc != objects[numColor].end(); ++itc){
			if((numColor==3) ||(numColor==4) || (numColor==5)){
			 	if(((z < (itc->zMax+0.1)) && (z > (itc->zMin-0.1)))
					&&((y < (itc->yMax+0.1)) && (y > (itc->yMin-0.1)))){

					entrado = true;
				}
			}else{
				if(((z < (itc->zMax+0.5)) && (z > (itc->zMin-0.5)))
				&&((y < (itc->yMax+0.5)) && (y > (itc->yMin-0.5)))){
					entrado = true;
				}
			}
		}
		if(!entrado){
			 //nuevo
			obj.mediaX = x;
			obj.mediaY = y;
			obj.mediaZ = z;
			obj.xMax = x;
			obj.yMax = y;
			obj.zMax = z;
			obj.xMin = x;
			obj.yMin = y;
			obj.zMin = z;
			obj.num_pixel = 1;
			objects[numColor].push_back(obj);
		}else{
			 
			for(itc = objects[numColor].begin(); itc != objects[numColor].end(); ++itc){
				//objects[MAGENTA]=lista.search_by_pos(i);
				if((numColor==3) ||(numColor==4) || (numColor==5)){
				 	if(((z < (itc->zMax+0.1)) && (z > (itc->zMin-0.1)))
						&&((y < (itc->yMax+0.1)) && (y > (itc->yMin-0.1)))){

						//lo modifico por que es el mismo objeto
						itc->mediaX = (itc->mediaX + x);
						itc->mediaY = (itc->mediaY + y);
						itc->mediaZ = (itc->mediaZ + z);
						if(x > itc->xMax){
								itc->xMax = x;
							} else{
								itc->xMin = x;
							}
							if(y > itc->yMax){
								itc->yMax = y;
							} else{
								itc->yMin = y;
							}

							if(z > itc->zMax){
								itc->zMax = z;
							} else{
								itc->zMin = z;

							}
						itc->num_pixel += 1;
						}
				}else{
					if(((z < (itc->zMax+0.5)) && (z > (itc->zMin-0.5)))
					&&((y < (itc->yMax+0.5)) && (y > (itc->yMin-0.5)))){

					//lo modifico por que es el mismo objeto
					itc->mediaX = (itc->mediaX + x);
					itc->mediaY = (itc->mediaY + y);
					itc->mediaZ = (itc->mediaZ + z);
					if(x > itc->xMax){
							itc->xMax = x;
						} else{
							itc->xMin = x;
						}
						if(y > itc->yMax){
							itc->yMax = y;
						} else{
							itc->yMin = y;
						}

						if(z > itc->zMax){
							itc->zMax = z;
						} else{
							itc->zMin = z;

						}
						itc->num_pixel += 1;
					}

				}
			}
		}
	}
}

void tresD::borrar(){
	for (int i = 0; i < NUM_COLORINES; ++i){
		while (!objects[i].empty()){
			objects[i].pop_front();
		}
	}
	//printf("BORRAMOS");
}

void tresD::seleccionar(){
	std::list<ColorObject>::iterator itc;
	int i =0;
	float x=0.0;
	float y=0.0;
	float z=0.0;
	int rojo, amarillo, azul, magenta, naranga, naranja_claro;
	for(int k=0; k<NUM_COLORINES;k++){
		for(itc = objects[k].begin(); itc != objects[k].end(); ++itc){


			if(itc->num_pixel > 100){
				i++;
				printf("num object %i: %i\n",k,i);
				std::cout<<"xMax: "<<itc->xMax<<std::endl;
				std::cout<<"yMax: "<<itc->yMax<<std::endl;
				std::cout<<"zMax: "<<itc->zMax<<std::endl;
				std::cout<<"xMin: "<<itc->xMin<<std::endl;
				std::cout<<"yMin: "<<itc->yMin<<std::endl;
				std::cout<<"zMin: "<<itc->zMin<<std::endl;

				x = (itc->mediaX) / (float)(itc->num_pixel);
				y = (itc->mediaY) / (float)(itc->num_pixel);
				z = (itc->mediaZ) / (float)(itc->num_pixel);

				std::cout<<"mediaX: "<<x<<std::endl;
				std::cout<<"mediaY: "<<y<<std::endl;
				std::cout<<"mediaZ: "<<z<<std::endl;

				std::cout<<"-----------"<<std::endl;
			}
		}
		
		i=0;
	}
}

/*
* Con esta funcion localizamos si es una baliza y la publicaremos con un tf
*/
void tresD::es_baliza(){

	std::list<ColorObject>::iterator itc;
	std::list<ColorObject>::iterator itc2;

	// std::list<ColorObject>::iterator itc3;
	// std::list<ColorObject>::iterator itc4;


	tf::StampedTransform R2Lm1, R2Lm2, R2Lm3, R2Lm4, R2Port1, R2Port2;

	R2Lm1.frame_id_ = "/base_link";
	R2Lm1.child_frame_id_ = "/perceived_lm1";
	R2Lm1.stamp_ = ros::Time::now() + ros::Duration(1.0);

	R2Lm2.frame_id_ = "/base_link";
	R2Lm2.child_frame_id_ = "/perceived_lm2";
	R2Lm2.stamp_ = ros::Time::now() + ros::Duration(1.0);

	R2Lm3.frame_id_ = "/base_link";
	R2Lm3.child_frame_id_ = "/perceived_lm3";
	R2Lm3.stamp_ = ros::Time::now() + ros::Duration(1.0);

	R2Lm4.frame_id_ = "/base_link";
	R2Lm4.child_frame_id_ = "/perceived_lm4";
	R2Lm4.stamp_ = ros::Time::now() + ros::Duration(1.0);

	R2Port1.frame_id_ = "/base_link";
	R2Port1.child_frame_id_ = "/perceived_port1";
	R2Port1.stamp_ = ros::Time::now() + ros::Duration(1.0);

	R2Port2.frame_id_ = "/base_link";
	R2Port2.child_frame_id_ = "/perceived_port2";
	R2Port2.stamp_ = ros::Time::now() + ros::Duration(1.0);

	float x=0.0;
	float y=0.0;
	float z=0.0;
	
	float x2=0.0;
	float y2=0.0;
	float z2=0.0;

	for(itc = objects[BLUE].begin(); itc != objects[BLUE].end(); itc++){
		x = (itc->mediaX) / (float)(itc->num_pixel);
		y = (itc->mediaY) / (float)(itc->num_pixel);
		z = (itc->mediaZ) / (float)(itc->num_pixel);
		for(itc2 = objects[MAGENTA].begin(); itc2 != objects[MAGENTA].end(); itc2++){
			x2 = (itc2->mediaX) / (float)(itc2->num_pixel);
			y2 = (itc2->mediaY) / (float)(itc2->num_pixel);
			z2 = (itc2->mediaZ) / (float)(itc2->num_pixel);

			if((z > z2) && (fabs(x-x2) < 0.30)&&((itc->num_pixel > 100) 
				&& (itc2->num_pixel > 100)) && (itc->num_pixel<2000)){
				// 	ROS_INFO("Baliza azul y magenta");
				R2Lm4.setOrigin(tf::Vector3(x, y, z));
				R2Lm4.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
				std::cout<<"LM4"<<std::endl;
				tfB.sendTransform(R2Lm4);
			}else if((z < z2) && (fabs(x-x2) < 0.30) && ((itc->num_pixel > 100) 
				&& (itc2->num_pixel > 100))&& (itc->num_pixel<2000)){
				// 	ROS_INFO("Baliza magenta y azul");
				R2Lm1.setOrigin(tf::Vector3(x, y, z));
				R2Lm1.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
				tfB.sendTransform(R2Lm1);
				std::cout<<"LM1"<<std::endl;

			}else if(itc->num_pixel > 3500){
				// 	ROS_INFO("Porteria azul");
				R2Port1.setOrigin(tf::Vector3(x, y, z));
				R2Port1.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
				std::cout<<"PORt1"<<std::endl;
				tfB.sendTransform(R2Port1);
			}
		}
	}

	for(itc = objects[YELLOW].begin(); itc != objects[YELLOW].end(); itc++){
		x = (itc->mediaX) / (float)(itc->num_pixel);
		y = (itc->mediaY) / (float)(itc->num_pixel);
		z = (itc->mediaZ) / (float)(itc->num_pixel);
		for(itc2 = objects[BLUE].begin(); itc2 != objects[BLUE].end(); itc2++){
			x2 = (itc2->mediaX) / (float)(itc2->num_pixel);
			y2 = (itc2->mediaY) / (float)(itc2->num_pixel);
			z2 = (itc2->mediaZ) / (float)(itc2->num_pixel);
			if((z > z2) && (fabs(x-x2) < 0.30) && ((itc2->num_pixel > 100) &&
				(itc->num_pixel > 100))&& (itc2->num_pixel<2000)&& (itc->num_pixel<2000)){
				// 	ROS_INFO("Baliza amarilla y azul");
				R2Lm3.setOrigin(tf::Vector3(x, y, z));
				R2Lm3.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
				tfB.sendTransform(R2Lm3);

			}else if(itc->num_pixel > 3500){
				// 	ROS_INFO("Porteria amarilla");
				R2Port2.setOrigin(tf::Vector3(x, y, z));
				R2Port2.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
				tfB.sendTransform(R2Port2);
				std::cout<<"PORT2"<<std::endl;
			}
		}
		for(itc2 = objects[MAGENTA].begin(); itc2 != objects[MAGENTA].end(); itc2++){
			x2 = (itc2->mediaX) / (float)(itc2->num_pixel);
			y2 = (itc2->mediaY) / (float)(itc2->num_pixel);
			z2 = (itc2->mediaZ) / (float)(itc2->num_pixel);
			if((z > z2) && (fabs(x-x2) < 0.30)&& ((itc2->num_pixel > 100) 
				&& (itc->num_pixel > 100))&& (itc->num_pixel<2000)){
				// 	ROS_INFO("Baliza amarilla y magenta");
				R2Lm2.setOrigin(tf::Vector3(x2, y2, z2));
				R2Lm2.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
				tfB.sendTransform(R2Lm2);
				std::cout<<"LM2"<<std::endl;


			}

		}
	}
}

/*
* Con esta funcion comprobamos si es una pelota y la enviamos por un tf
*/
void tresD::es_pelota(){
	std::list<ColorObject>::iterator itc;

	tf::StampedTransform R2red, R2orange, R2light_orange;
	bool pelota_roja = false;
	bool pelota_naranja = false;
	bool pelota_light_naranja = false;

	R2red.frame_id_ = "/base_link";
	R2red.child_frame_id_ = "/ball_red";
	R2red.stamp_ = ros::Time::now() + ros::Duration(1.0);

	R2orange.frame_id_ = "/base_link";
	R2orange.child_frame_id_ = "/ball_orange";
	R2orange.stamp_ = ros::Time::now() + ros::Duration(1.0);

	R2light_orange.frame_id_ = "/base_link";
	R2light_orange.child_frame_id_ = "/ball_light_orange";
	R2light_orange.stamp_ = ros::Time::now() + ros::Duration(1.0);

	float x=0.0;
	float y=0.0;
	float z=0.0;
	for(itc = objects[RED].begin(); itc != objects[RED].end(); itc++){
		//miramos que sea una pelota y al final solo mandaremos la posicion de la pelota que este mas cerca
		if(itc->num_pixel > 40){
			/// 	ROS_INFO("Pelota Roja);
			x = (itc->mediaX) / (float)(itc->num_pixel);
			y = (itc->mediaY) / (float)(itc->num_pixel);
			z = (itc->mediaZ) / (float)(itc->num_pixel);

			pelota_roja = true;
			R2red.setOrigin(tf::Vector3(x, y, z));
			R2red.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
			std::cout<<"red"<<std::endl;
		}
		if(pelota_roja){
			tfB.sendTransform(R2red);
			pelota_roja=false;
		}
	}
	
	x=0.0;
	y=0.0;
	z=0.0;

	for(itc = objects[ORANGE].begin(); itc != objects[ORANGE].end(); itc++){
		if(itc->num_pixel > 40){
			/// 	ROS_INFO("Pelota Naranja);
			x = (itc->mediaX) / (float)(itc->num_pixel);
			y = (itc->mediaY) / (float)(itc->num_pixel);
			z = (itc->mediaZ) / (float)(itc->num_pixel);
			pelota_naranja=true;
			R2orange.setOrigin(tf::Vector3(x, y, z));
			R2orange.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
			std::cout<<"ORANGE"<<std::endl;

		}
		if (pelota_naranja){
			tfB.sendTransform(R2orange);
			pelota_naranja=false;
		}
	}
	
	x=0.0;
	y=0.0;
	z=0.0;

	for(itc = objects[LIGHT_ORANGE].begin(); itc != objects[LIGHT_ORANGE].end(); itc++){
		if(itc->num_pixel > 40){
			/// 	ROS_INFO("Pelota light_orange);
			x = (itc->mediaX) / (float)(itc->num_pixel);
			y = (itc->mediaY) / (float)(itc->num_pixel);
			z = (itc->mediaZ) / (float)(itc->num_pixel);
			pelota_light_naranja=true;
			R2light_orange.setOrigin(tf::Vector3(x, y, z));
			R2light_orange.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
			std::cout<<"light_orange"<<std::endl;

		}
		if (pelota_light_naranja){
			tfB.sendTransform(R2light_orange);
			pelota_light_naranja=false;
		}
	}
}



void tresD::imageCb(const ros::MessageEvent<sensor_msgs::PointCloud2 const>& msg){

	//std::cerr<<"entro"<<std::endl;

	actualizar = false;
	sensor_msgs::PointCloud2 in_basefoot;

	pcl_ros::transformPointCloud("/base_link", *(msg.getMessage()), in_basefoot, tfL);
	pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb, PCxyzrgbout;

	pcl::fromROSMsg(in_basefoot, PCxyzrgb);
	PCxyzrgbout = PCxyzrgb;
	PCxyzrgbout.clear();

	int a=0;
	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
	for (it = PCxyzrgb.begin(); it != PCxyzrgb.end(); ++it) {
		pcl::PointXYZHSV hsv;
		pcl::PointXYZRGBtoXYZHSV(*it, hsv);

		if ((it->x == it->x)&&(it->y == it->y)&&(it->z == it->z)){
			if (((hsv.h >= hlower_magenta) && (hsv.h <= hupper_magenta)) 
				&& ((hsv.s >=nslower) && (hsv.s <= nsupper))
				&& ((hsv.v >= nvlower)&& (hsv.v <= nvupper))){
				//ROS_INFO("MAGENTA");
				PCxyzrgbout.push_back(*it);
				tresD::update(MAGENTA, (float) it->x, (float) it->y, (float) it->z);

			}else if (((hsv.h >= hlower_blue) && (hsv.h <= hupper_blue)) 
				&& ((hsv.s >=slower_blue) && (hsv.s <= supper_blue))
				&& ((hsv.v >= vlower_blue)&& (hsv.v <= vupper_blue))){
				//ROS_INFO("BLUE");

				PCxyzrgbout.push_back(*it);
				tresD::update(BLUE, (float) it->x, (float) it->y, (float) it->z);				

			}else if ((((hsv.h >= hlower_yellow) && (hsv.h <= hupper_yellow)) 
				&& ((hsv.s >=slower_yellow) && (hsv.s <= supper_yellow))
				&& ((hsv.v >= vlower_yellow)&& (hsv.v <= vupper_yellow)))||
				(((hsv.h >= 0.0) && (hsv.h <= 87.0)) 
				&& ((hsv.s >=88.0/255.0f) && (hsv.s <= 155.0/255.0f))
				&& ((hsv.v >= 151.0/255.0f)&& (hsv.v <= 224.0/255.0f)))){
				PCxyzrgbout.push_back(*it);
				tresD::update(YELLOW, (float) it->x, (float) it->y, (float) it->z);
				//std::cout<<"YELLOW 1"<<std::endl;

			}else if (((hsv.h >= hlower_light_orange) && (hsv.h <= hupper_light_orange)) 
				&& ((hsv.s >=slower_light_orange) && (hsv.s <= supper_light_orange))
				&& ((hsv.v >= vlower_light_orange)&& (hsv.v <= vupper_light_orange))){
					//ROS_INFO("LIGHT_ORANGE");
				PCxyzrgbout.push_back(*it);
				tresD::update(LIGHT_ORANGE, (float) it->x, (float) it->y, (float) it->z);

			}else if (((hsv.h >= hlower_red) && (hsv.h <= hupper_red)) 
				&& ((hsv.s >=slower_red) && (hsv.s <= supper_red))
				&& ((hsv.v >= vlower_red)&& (hsv.v <= vupper_red))){
				if(((hsv.h >= hlower_orange) && (hsv.h <= hupper_orange)) 
				&& ((hsv.s >=slower_orange) && (hsv.s <= supper_orange))
				&& ((hsv.v >= vlower_orange)&& (hsv.v <= vupper_orange))){
					//ROS_INFO("ORANGE");
					PCxyzrgbout.push_back(*it);
					tresD::update(ORANGE, (float) it->x, (float) it->y, (float) it->z);
				}else{
					//	ROS_INFO("RED");
					PCxyzrgbout.push_back(*it);
					tresD::update(RED, (float) it->x, (float) it->y, (float) it->z);
				}

			}else {
				it->r = 0;
				it->g = 0;
				it->b = 0;
			
			}
		}

	}

	//tresD::seleccionar();
	tresD::es_baliza();
	tresD::es_pelota();

	tresD::borrar();



	//tansform PCxyzrgb (pcl::PointCloud<pcl::PointXYZRGB>) to Image to display in the OpenCV window
	pcl::toROSMsg(PCxyzrgb, in_basefoot);

	sensor_msgs::Image image;
	//cv_bridge::CvImagePtr cv_imageout;

	pcl::toROSMsg(in_basefoot, image);


	float x, y, z;
	x = y = z = 0.0;
	int c = 0;

	for (it = PCxyzrgbout.begin(); it != PCxyzrgbout.end(); ++it) {

		if (it->x == it->x) //This seems to be the only way to detect if it is NaN
				{
			x = x + it->x;
			y = y + it->y;
			z = z + it->z;
			c++;
		}
	}

	if (c > 0) {
		x = x / (float) c;
		y = y / (float) c;
		z = z / (float) c;
	}

	sensor_msgs::PointCloud2 pcout;
	pcl::toROSMsg(PCxyzrgbout, pcout);
	image_pub_.publish(in_basefoot);
	
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter3D");

	tresD ic;

	ros::spin();

	return 0;
}
