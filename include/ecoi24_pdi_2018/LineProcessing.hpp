/*
 * LineProcessing.hpp
 *
 *  Created on: Oct 26, 2018
 *      Author: giovani
 */

#ifndef SRC_LINEPROCESSING_HPP_
#define SRC_LINEPROCESSING_HPP_

#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <boost/thread.hpp>
#include "controller.h"
#include "ErleRoverManager.hpp"
#include <opencv2/opencv.hpp>



/**

class to receive the handle of ros with the image
**/
class LineProcessing{

private:

	ros::NodeHandle *n;

	int width_image;
   	int heigh_image;

	double velocity;
   	double ex_ant;
   	double ex_sum;
   	double t_ant;
   	double kp,ki,kd;
	
	ErleRoverManager erlerover_manager;
	ros::Publisher rc_override_pub;
	
	void SendCommandEB2();
 	void lineExtraction(const sensor_msgs::ImageConstPtr& msg);
	double visual_servoing_control(cv::Point2f statePt1, cv::Point2f statePt2 );

public:
	/**
	@brief constructor of class with initi with default parameters
	@TODO: use rosparam to configure the parameters
	**/
	LineProcessing(ros::NodeHandle nh);
	~LineProcessing();

	/**
	@brief
	ROS callback that return the image from the topic
	@param msg sensor_msgs::Image

	**/
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};



#endif /* SRC_LINEPROCESSING_HPP_ */
