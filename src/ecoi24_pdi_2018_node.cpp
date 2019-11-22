#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "LineProcessing.hpp"


//to tun the application:  rosrun ecoi24_pdi_2018 ecoi24_pdi_2018_node _image_transport:=compressed
//		      or:  rosrun ecoi24_pdi_2018 ecoi24_pdi_2018_node


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "ecoi24_pdi_2018");
  	ros::NodeHandle nh;

  	image_transport::ImageTransport it(nh);
  	LineProcessing myLine(nh);
	

   	image_transport::Subscriber sub = it.subscribe("/camera/image", 1,boost::bind(&LineProcessing::imageCallback, &myLine, _1));
	//image_transport::Subscriber sub = it.subscribe("/raspicam_node/image_raw", 1,boost::bind(&LineProcessing::imageCallback, &myLine, _1));
   	ros::spin();

  	return 0;
}
