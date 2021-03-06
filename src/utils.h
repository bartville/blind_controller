#pragma once
#include <iostream>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <vector>

struct Action{
  enum Frame{Idle, Motion, ChangeDirection};
  Frame frame;
  geometry_msgs::Twist twist;
  float meter;
  float rad;
  float travelled_rad;
  Action() {
    frame = Frame::Idle;
    meter = 0.f;
    rad = 0.f;
    travelled_rad = 0.f;
  }
};
typedef std::vector<Action> ActionVector;


struct RobotPose{
  float x;
  float y;
  float yaw;
  RobotPose(){
    x = 0.f;
    y = 0.f;
    yaw = 0.f;
  }
};

float euclideanDistance(const RobotPose& a,
			const RobotPose& b){
  return fabs(sqrt( (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) ));
}


float radiansDifference(const RobotPose& a,
			const RobotPose& b){
  float difference = a.yaw-b.yaw;
  if(difference > M_PI) {
    difference -= 2*M_PI;
  }
  if(difference < -M_PI) {
    difference += 2*M_PI;
  }
  return fabs(difference);
}


bool checkTerminationCondition(Action& action,
			       const RobotPose& start,
			       RobotPose& last,
			       const RobotPose& current){
  if(action.frame == Action::Frame::Motion){
    float travelled_distance = euclideanDistance(start, current);
    if(travelled_distance > action.meter)
      return true;
  }
  else if(action.frame == Action::Frame::ChangeDirection){    
    action.travelled_rad += radiansDifference(last, current);
    last = current;
    std::cerr << "trav rad: " << action.travelled_rad << std::endl;
    if(action.travelled_rad > action.rad) {
      return true;
    }
  }

  return false;
}
  
void deg2rad(float& rad, const float& deg){
  rad = deg*M_PI/180.f;
}

float yawFromQuaternion(const float& qw,
			const float& qx,
			const float& qy,
			const float& qz) {
  return atan2(2.0 * (qw * qz + qx * qy),
	       1.0 - 2.0 * (qy * qy + qz * qz));
}


void printBanner(const char** banner) {
    const char** b = banner;
    while(*b) {
        std::cerr << *b << std::endl;
        b++;
    }
}
