#pragma once

#include <iostream>
#include <geometry_msgs/Twist.h>

class BlindController{

  BlindController();
  
  ~BlindController(){
    /* nothing to be destroyed */
  }

  void changeDirectionCallback(const blind_controller::change_direction::ConstPtr& msg);
  void motionCallback(const blind_controller::motion::ConstPtr& msg);
  

}

