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
int contVel=0;

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

	Mat original_img, cropped_img, cropped_img_2, gray_img, filter_img, threshold_img, blur_img,canny_img, houghline_res, colored_canny, median_img;
	original_img = cv_ptr->image; //imagem original 
	int altura1 = 20;
	int altura2 = 40;
	int tamanho = 20;
	Rect tam(altura1, heigh_image-(altura1+tamanho), width_image-20, 20); //tamanho da imagem que vai ser cortado
	cropped_img = original_img(tam); //imagem cortada

	// Rect tam(20, heigh_image-60, width_image-40, 30); //tamanho da imagem que vai ser cortado
	// cropped_img = original_img(tam); //imagem cortada

	imshow("original", original_img);
	cvtColor(cropped_img, gray_img, CV_BGR2GRAY); //escala de cinza (função canny só funciona com escala de cinza)
	imshow("gray", gray_img);
	threshold(gray_img, threshold_img, 70, 255, THRESH_BINARY_INV);
	medianBlur(threshold_img, median_img, 9);
	imshow("median", median_img);
	Canny(median_img, canny_img, 100, 200, 3); //detecta a borda da linha
	imshow("canny", canny_img);


	if(coord_ant[0]==0){
		coord_ant[0] = 172;
		coord_ant[1] = 20;
		coord_ant[2] = 172;
		coord_ant[3] = 0;
	}

	cvtColor(canny_img, colored_canny, CV_GRAY2BGR);

	vector<Vec4i> lines;
	Vec4i coord;
	coord[0] = 0;
	coord[1] = 0;
	coord[2] = 0;
	coord[3] = 0;

	HoughLinesP(canny_img, lines, 1, CV_PI/180, 20, 5, 20 ); //detecta os pontos
	int cont = 0;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		line( colored_canny, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3, LINE_AA);
		if(abs(l[3]-l[1]) > 5){
			coord[0]+= l[0];
			coord[1] += l[1];
			coord[2] += l[2];
			coord[3] += l[3];
			cont++;
		}
	}
	
	if (cont!=0){
		coord[0] /= cont;
		coord[1] /= cont;
		coord[2] /= cont;
		coord[3] /= cont;
		coord_ant = coord;
	}
	else{
		coord = coord_ant;	
	}
	line( colored_canny, Point(coord[0], coord[1]), Point(coord[2], coord[3]), Scalar(0,255,0), 3, LINE_AA);

	namedWindow( "saida", WINDOW_AUTOSIZE );
	imshow("saida", colored_canny);
	//namedWindow( "saidaFinal", WINDOW_AUTOSIZE );
	//imshow("saidaFinal", original_img);

	
	int x = (int)(coord[0]+coord[2])/2;
	int w_image = (int)(width_image-40);
	int yaw = 1500;

	//int yaw = 1500 + 500*(x - w_image/2)/2;
	//cout<<"YAW "<<yaw<<endl;
	int velocidade;

	//int razao = 0;
	//razao = int(int(abs(coord[0]-coord[2])/abs(coord[1]-coord[3]))*10000);

	// if (x<w_image){
	// 	yaw = -1500*(w_image-x)/w_image;
	// }
	// else if(x>w_image){
	// 	yaw = 1500*(x-w_image)/w_image;
	// }
	// contVel++;
	velocidade = 1565;
	// if (contVel%2==0) velocidade--;
	// else velocidade++;
	
	
	//float angulo;
	if (x<w_image){
		yaw = -1000*(w_image-x)/w_image;
	}
	else if(x>w_image){
		yaw = 1000*(x-w_image)/w_image;
	}
	else yaw = 0;
	cout<<"YAW "<<yaw<<endl;
	yaw = yaw+2000;
	//cout<<"YAW "<<yaw<<endl;
	if (yaw > 1900){
		yaw = 1900;
	}
	else if (yaw < 1100){
		yaw = 1100;
	}


	//cout << float(float(coord[3]-coord[1])/float(coord[2]-coord[0])) << endl;
	

	cout<<"Center: "<< w_image/2 <<"\nPoint: "<<x<<"\nEstressamento: "<<yaw<<"\nVelocidade: "<<velocidade<<"\n\n";

	//cout<<"Angulo: "<<yaw<<endl;

	//codigo para mandar comandos para o carrinho
	//double driver_wheel = visual_servoing_control(ponto1, ponto2);
	// static int aux = 0;
	erlerover_manager.setAngularVelocity(yaw);
	erlerover_manager.setLinearVelocity(velocidade);
	 	//SendCommandEB2();

	waitKey(1);


	

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
	//cout<<"ENVIANDO COMANDO\n";
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

