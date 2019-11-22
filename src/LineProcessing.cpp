/*
 * LineProcessing.cpp
 *
 *  Created on: Oct 26, 2018
 *      Author: giovani
 */


#include <stdio.h>
#include "LineProcessing.hpp"
#include <vector>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <cmath>
#include <string>
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

Vec4i coord_ant;

/**
@brief constructor of class with initi with default parameters
**/
LineProcessing::LineProcessing(ros::NodeHandle nh){
   
	n = &nh;
   	erlerover_manager.init();

   	rc_override_pub = n->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

   	boost::thread t(boost::bind(&LineProcessing::SendCommandEB2, this));

   	cv::namedWindow("view");
   	cv::startWindowThread();
   
};


LineProcessing::~LineProcessing()
{

	cv::destroyWindow("view");
}



void LineProcessing::lineExtraction(const sensor_msgs::ImageConstPtr& msg)
{

	width_image=msg->width;
   	heigh_image=msg->height;
	

	//@bug it is not neceesary to convert to opencv image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Insert your code here to process LINE DETECTION on the image
	////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//cv_ptr->image
	// cv::mat gray;
	//cv::rgb2gray(cvptr->image, gray);
	//gray.at<float>(x,y);

	Mat original_img, cropped_img, gray_img, filter_img, threshold_img, blur_img,canny_img, houghline_res, colored_canny;
	original_img = cv_ptr->image; //imagem original 
	Rect tam(20, heigh_image-60, width_image-40, 30); //tamanho da imagem que vai ser cortado
	cropped_img = original_img(tam); //imagem cortada

	imshow("original", original_img);
	cvtColor(cropped_img, gray_img, CV_BGR2GRAY); //escala de cinza (função canny só funciona com escala de cinza)
	imshow("gray", gray_img);
	//threshold(gray_img, threshold_img, 70, 255, THRESH_BINARY_INV);
	Canny(gray_img, canny_img, 50, 200, 3); //detecta a borda da linha
	imshow("canny", canny_img);

	cvtColor(canny_img, colored_canny, CV_GRAY2BGR);
	
	vector<Vec4i> lines;
	Vec4i coord;
	coord[0] = 0;
	coord[1] = 0;
	coord[2] = 0;
	coord[3] = 0;
	HoughLinesP(canny_img, lines, 1, CV_PI/180, 20, 5, 20 ); //detecta os pontos
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		line( colored_canny, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3, LINE_AA);

		coord[0]+= l[0];
		coord[1] += l[1];
		coord[2] += l[2];
		coord[3] += l[3];
	}
	
	if (lines.size()!=0){
		coord[0] /= lines.size();
		coord[1] /= lines.size();
		coord[2] /= lines.size();
		coord[3] /= lines.size();	
	}


	if (abs(coord[1]-coord[3]) > 15 && abs((coord[0]+coord[2])/2-(coord_ant[0]+coord_ant[2])/2) < width_image/2){
		coord_ant = coord;
		line( colored_canny, Point(coord[0], coord[1]), Point(coord[2], coord[3]), Scalar(0,255,0), 3, LINE_AA);
	}
	else
	{
		coord = coord_ant;
		line( colored_canny, Point(coord[0], coord[1]), Point(coord[2], coord[3]), Scalar(0,255,0), 3, LINE_AA);
	}
	namedWindow( "saida", WINDOW_AUTOSIZE );
	imshow("saida", colored_canny);


	//cout << (180*atan((coord[2]-coord[0])/(coord[3]-coord[1])))/3.14159 << endl;
	int x = (int)(coord[0]+coord[2])/2;
	int w_image = (int)(width_image-40)/2;
	float angulo;
	if (x<w_image){
		angulo = -100*(w_image-x)/w_image;
	}
	else if(x>w_image){
		angulo = 100*(x-w_image)/w_image;
	}
	else angulo = 0;

	cout<<angulo<<endl;



	waitKey(3);


	

}

double LineProcessing::visual_servoing_control(cv::Point2f statePt1, cv::Point2f statePt2 )
{

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// VISUAL SERVOING CONTROL
	////////////////////////////////////////////////////////////////////////////////////////////////////

	std::cout << "pt1: " << statePt1 << "pt2: " << statePt2 << " Velocity: " << velocity << std::endl;

	//find the line equation
	double m = (statePt2.x - statePt1.x) / (statePt2.y - statePt1.y);
	double b = statePt1.x - m * statePt1.y;

	//cv::Point2f observed_pt(m*(*desired_line)(1) + b,(*desired_line)(1));

	
	cv::Mat error_control = (cv::Mat_<float>(2,1) << 0.0, 0.0); // error_x and error_theta

	float observed_angle = atan(m);

	//error_control.at<float>(0) =       (*desired_line)(0) -  observed_pt.x; // estimated X error
	//error_control.at<float>(1) = (atan((*desired_line)(2)) - observed_angle); // estimated theta error

	double lambda = 1;
	double pho = 30.0/180.0*M_PI; // radians
	double ty = 1.4f; // meters
	double tz = 2.2f; // meters
	double v = velocity;  // m/s
	double c_x = width_image/2.0f;
	double c_y = heigh_image/2.0f;

	//double delta=0;
	double delta = controller( 0.0, //(*desired_line)(4), // Theta reference
				   0.0, // (*desired_line)(0) - c_x,  // X point reference
				   0.0, //(*desired_line)(1) - c_y,  // Y point reference
				   error_control.at<float>(1), // theta error
				   error_control.at<float>(0), //  X error
				   lambda, // lambda paramenter of the controller
				  pho, // pho: tilt angle of the camera
				   ty, // y axis translation to camera reference
			   tz, // z axis translation to camera reference
			   v // speed...
			);
	
	
	double driver_wheel;	
	//driver_wheel = std::max(std::min(1.0,driver_wheel),-1.0) * -1.0f;
	
	//std::cout << "Time DeltaT (dt) : " << dt << "\n Desired_point: " << cv::Point2f((*desired_line)(0),(*desired_line)(1)) << " Desired Angle: " << atan((*desired_line)(2))/M_PI*180.0 << "\n  Observerd_pt: " << observed_pt << " Observed Angle: " << observed_angle/M_PI*180.0 << "\n ERROR (X): " << error_control(0) << " (theta) " << error_control(1)/M_PI*180 << "\n Delta: " << delta/M_PI*180 << " [degree] \n delta: " << delta << " [rad] "  << std::endl;

	//std::max(std::min(delta,(double)0.6),(double)-0.6);
	
	//return delta; // [rad]
	return 0.0;
}

void LineProcessing::SendCommandEB2()
{
	int rate = 30;
  	ros::Rate r(rate);

	mavros_msgs::OverrideRCIn msg_override;

	while (n->ok()){
		//std::cout<< " command send... " <<std::endl;	
		msg_override.channels[0] = erlerover_manager.getAngularVelocity();
		msg_override.channels[1] = 0;
		msg_override.channels[2] = erlerover_manager.getLinearVelocity();
		msg_override.channels[3] = 0;
		msg_override.channels[4] = 0;
		msg_override.channels[5] = 0;
		msg_override.channels[6] = 0;
		msg_override.channels[7] = 0;

		rc_override_pub.publish(msg_override);
		r.sleep();
	}
	
}


/**
@brief
ROS callback that return the image from the topic
@param msg sensor_msgs::Image

**/
void LineProcessing::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	
	  try
	  {
		 //std::cout<< " image callback... " <<std::endl;	  
		 lineExtraction(msg);

	  }
	  catch (cv_bridge::Exception& e)
	  {
	    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }
};

