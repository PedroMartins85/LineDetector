/*
 * ErleRoverManager.hpp
 *
 *  Created on: Oct 26, 2018
 *      Author: giovani
 */

#include <string>
#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>

class ErleRoverManager
{
public:
 
  ErleRoverManager();
  ~ErleRoverManager();
  bool init();

  int getLinearVelocity();
  int getAngularVelocity();
  void setLinearVelocity(int value) { vx = value;};
  void setAngularVelocity(int value){ wz = value;};
  

private:
  int vx, wz;

  int linear_vel_step, linear_vel_max, linear_vel_min;
  int angular_vel_step, angular_vel_max, angular_vel_min;
  std::string name;

  void incrementLinearVelocity();
  void decrementLinearVelocity();
  void incrementAngularVelocity();
  void decrementAngularVelocity();
  void resetVelocity();

  

};
