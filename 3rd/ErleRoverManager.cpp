


#include "ErleRoverManager.hpp"


int ErleRoverManager::getLinearVelocity()
{
  return vx;
}

int ErleRoverManager::getAngularVelocity()
{
  return wz;
}

/**
 * @brief Default constructor, needs initialisation.
 */
ErleRoverManager::ErleRoverManager() :
                         linear_vel_step(2),
                         linear_vel_max(1860), // 1560
                         linear_vel_min(1140), // 1440
                         angular_vel_step(100),
                         angular_vel_max(1900),
                         angular_vel_min(1100),
                         vx(0.0), wz(0.0)
{
  
}

ErleRoverManager::~ErleRoverManager()
{
 
}

/**
 * @brief Initialises the node.
 */
bool ErleRoverManager::init()
{
  std::cout << "ErleRoverManager : using linear  vel step [" << linear_vel_step << "]." << std::endl;
  std::cout << "ErleRoverManager : using linear  vel max  [" << linear_vel_max << ", " << linear_vel_min << "]." << std::endl;
  std::cout << "ErleRoverManager : using angular vel step [" << angular_vel_step << "]." << std::endl;
  std::cout << "ErleRoverManager : using angular vel max  [" << angular_vel_max << ", " << angular_vel_min << "]." << std::endl;

  vx = 1500;
  wz = 1500;

  return true;
}


/**
 * @brief If not already maxxed, increment the command velocities..
 */
void ErleRoverManager::incrementLinearVelocity()
{
  if (vx <= linear_vel_max){
    vx += linear_vel_step;
  }
}

/**
 * @brief If not already minned, decrement the linear velocities..
 */
void ErleRoverManager::decrementLinearVelocity()
{
  if (vx >= linear_vel_min){
    vx -= linear_vel_step;
  }
}

/**
 * @brief If not already maxxed, increment the angular velocities..
 */
void ErleRoverManager::incrementAngularVelocity()
{
  if (wz <= angular_vel_max){
    wz += angular_vel_step;
  }
}

/**
 * @brief If not already mined, decrement the angular velocities..
 */
void ErleRoverManager::decrementAngularVelocity()
{
  if (wz >= angular_vel_min){
    wz -= angular_vel_step;
  }
}

void ErleRoverManager::resetVelocity()
{
  vx = 1500;
  wz = 1500;
}


