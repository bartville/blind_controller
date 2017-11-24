#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <blind_controller/Motion.h>
#include <blind_controller/ChangeDirection.h>
#include <nav_msgs/Odometry.h>

#include "utils.h"

RobotPose robot_pose, last_robot_pose;
ActionVector action_vector;

float translation_speed = 0.1; // 0.1 meter/second
float rotational_speed = 0.1; // 0.1 rad/second
float motion_default_value = 0.5; // default value 0.5m
float change_direction_default_value = 1.5708; // default value 90 degrees

bool firstOdom = true;

void odometryCallback(const nav_msgs::OdometryConstPtr &msg){

  // update robot 2D pose
  robot_pose.x = msg->pose.pose.position.x;
  robot_pose.y = msg->pose.pose.position.y;
  robot_pose.yaw = yawFromQuaternion(msg->pose.pose.orientation.w,
				     msg->pose.pose.orientation.x,
				     msg->pose.pose.orientation.y,
				     msg->pose.pose.orientation.z);
  //std::cerr << "o";

  if(firstOdom) {
    firstOdom = false;
    last_robot_pose = robot_pose;
  }

}

void motionCallback(const blind_controller::Motion::ConstPtr &msg){
  std::cerr << "\n[motionCallback]: received message "<< std::endl;
  std::string direction = msg->direction;
  float meter = msg->distance;
  std::cerr << "dir: " << direction << "  di " << meter << " metri" << std::endl;

  Action new_action;
  new_action.frame = Action::Frame::Motion;
  geometry_msgs::Twist new_twist;

  if(!direction.compare("avanti")){
    new_twist.linear.x = translation_speed;
  }
  else if(!direction.compare("indietro")){
    new_twist.linear.x = -translation_speed;
  }
  else{
    std::cerr << "wrong direction! This message will be destroyed!" << std::endl;
    return;
  }
  new_action.twist = new_twist;
  
  new_action.meter = motion_default_value;
  if (meter > 1e-6) {
    new_action.meter = meter;
  }
  action_vector.push_back(new_action);
}

void changeDirectionCallback(const blind_controller::ChangeDirection::ConstPtr &msg){
  std::cerr << "\n[changeDirectionCallback]: received message "<< std::endl;
  std::string direction = msg->direction;
  float deg = msg->angle;
  float rad;
  deg2rad(rad, deg);
  std::cerr << "dir: " << direction << "  di " << deg << " gradi (" << rad << " rad)" << std::endl;

  Action new_action;
  new_action.frame = Action::Frame::ChangeDirection;
  geometry_msgs::Twist new_twist;

  if(!direction.compare("sinistra")){
    new_twist.angular.z = rotational_speed;
  }
  else if(!direction.compare("destra")){
    new_twist.angular.z = -rotational_speed;    
  }
  else{
    std::cerr << "wrong direction! This message will be destroyed!" << std::endl;
    return;
  }
  new_action.twist = new_twist;

  new_action.rad = change_direction_default_value;
  if (rad > 1e-6) {
    new_action.rad = rad;
  }
  action_vector.push_back(new_action);
}


const char* banner[]={
  "\n\nUsage:  blind_controller_node -twist_topic <twist_topic> -odom-topic <odom_topic> -motion-topic <motion_topic> -change-direction-topic <change_direction_topic> [Options]\n",
  "Example:  blind_controller_node -odom-topic /odom -motion-topic /motion -change-direction-topic /change_direction \n\n",
  "Options:\n",
  "------------------------------------------\n",
  "-odom-topic <string>             topic name of odometry of the platform, default: /odom",
  "-twist-topic <string>            topic name of twist to control the platform, default: /cmd_vel",
  "-motion-topic <string>           topic name of motion command, default: /motion",
  "-change-direction-topic <string> topic name of change-direction command, default: /change_direction",
  "-max-trans-speed <float>         maximum translation speed, default 0.1 [m/s]",
  "-max-rot-speed <float>           maximum translation speed, default 0.1 [rad/s]",
  "-idle-time <float>               idle/sleep time between consecutive actions, default 0.f",
  "-h                               this help\n",
  0
};

int main(int argc, char** argv){
  
  ros::init(argc, argv, "BlindControllerNode");
  ros::NodeHandle nh("~");
  ROS_INFO("BlindControllerNode started!");

  if(argc < 1){
    printBanner(banner);
    return 1;
  }

  // default values
  std::string odom_topic = "/odom"; 
  std::string twist_topic = "/cmd_vel";
  std::string motion_topic = "/motion";
  std::string change_direction_topic = "/change_direction";
  float idle_time = 0.f;
  
  int c=1;

  while(c<argc){
    if (! strcmp(argv[c],"-h")){
      printBanner(banner);
      return 1;
    }
    else if(! strcmp(argv[c],"-odom-topic")){
      c++;
      odom_topic = argv[c];
    }
    else if(! strcmp(argv[c],"-twist-topic")){
      c++;
      twist_topic = argv[c];
    }
    else if(! strcmp(argv[c],"-motion-topic")){
      c++;
      motion_topic = argv[c];
    }
    else if(! strcmp(argv[c],"-change-direction-topic")){
      c++;
      change_direction_topic = argv[c];
    }
    else if(! strcmp(argv[c],"-max-trans-speed")){
      c++;
      translation_speed = std::atof(argv[c]);
    }
    else if(! strcmp(argv[c],"-max-rot-speed")){
      c++;
      rotational_speed = std::atof(argv[c]);
    }
    else if(! strcmp(argv[c],"-idle-time")){
      c++;
      idle_time = std::atof(argv[c]);
    }
    c++;
  }
  
 
  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odometryCallback);

  ros::Subscriber motion_sub = nh.subscribe(motion_topic, 10, motionCallback);
  ros::Subscriber change_dir_sub = nh.subscribe(change_direction_topic, 10, changeDirectionCallback);
  
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(twist_topic, 10);
  ros::Rate loop_rate(30);

  enum State{IDLE, EXECUTING};

  action_vector.clear();
  State current_state = State::IDLE;

  geometry_msgs::Twist stop_msg; //an empty msg used to stop the robot
  
  Action current_action;
  RobotPose action_starting_pose;
  
  while(ros::ok()){

    // if the robot is in Executing mode,
    // check for the end of action
    if(current_state == State::EXECUTING) {
      bool terminated = checkTerminationCondition(current_action,
						  action_starting_pose,
						  last_robot_pose,
						  robot_pose);
      if(terminated){                             // if the action is completed
	vel_pub.publish(stop_msg);                // stop the robot
	current_state = State::IDLE;              // change state to IDLE
      }else{                                      // otherwise
	vel_pub.publish(current_action.twist);    // continue moving
      }      
    }

    
    // if the robot is in Idle, check for other
    // motions in the twist vector
    if(current_state == State::IDLE) {
      if(action_vector.size()){  	                    // if there are new action
	current_action = action_vector.front();             // pop first element
	action_vector.erase(action_vector.begin());         // erase first element
	current_state = State::EXECUTING;                   // change state to EXECUTING
	action_starting_pose = robot_pose;                  // reset starting pose
	usleep(idle_time*1e9); //to microsec
      }
      vel_pub.publish(stop_msg);
    }   
    
    ros::spinOnce();
    loop_rate.sleep();
    
  }

  return 0;
}
