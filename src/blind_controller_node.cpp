#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <blind_controller/Motion.h>
#include <blind_controller/ChangeDirection.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include "utils.h"

RobotPose robot_pose, last_robot_pose;
ActionVector action_vector;

ros::Publisher* ack_pub;

const std::string ack_msg = "ACK";
const std::string bad_ack_msg = "BADMSG";

float translation_speed;
float rotational_speed;
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
  std::string speed = msg->speed;
  float meter = msg->distance;
  std::cerr << "dir: " << direction << "  di " << meter << " metri" << std::endl;

  Action new_action;
  new_action.frame = Action::Frame::Motion;
  geometry_msgs::Twist new_twist;

  int speed_scale = 1;
  if(!speed.compare("velocemente"))
    speed_scale *=2;
  else if(!speed.compare("lentamente"))
    speed_scale /=2;


  if(!direction.compare("avanti") || !direction.compare("dritto")){
    new_twist.linear.x = translation_speed*speed_scale;
  }
  else if(!direction.compare("indietro")){
    new_twist.linear.x = -translation_speed*speed_scale;
  }
  else{
    std::cerr << "wrong direction! This message will be destroyed!" << std::endl;
    ack_pub->publish(bad_ack_msg);
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
  std::string speed = msg->speed;
  float deg = msg->angle;
  float rad;
  deg2rad(rad, deg);
  std::cerr << "dir: " << direction << "  di " << deg << " gradi (" << rad << " rad)" << std::endl;

  Action new_action;
  new_action.frame = Action::Frame::ChangeDirection;
  geometry_msgs::Twist new_twist;


  int speed_scale = 1;
  if(!speed.compare("velocemente"))
    speed_scale *=2;
  else if(!speed.compare("lentamente"))
    speed_scale /=2;


  if(!direction.compare("sinistra")){
    new_twist.angular.z = rotational_speed*speed_scale;
  }
  else if(!direction.compare("destra")){
    new_twist.angular.z = -rotational_speed*speed_scale;    
  }
  else{
    std::cerr << "wrong direction! This message will be destroyed!" << std::endl;
    ack_pub->publish(bad_ack_msg);
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
  "\n\nUsage:  blind_controller_node _twist_topic:= <twist_topic> _odom_topic:= <odom_topic> _motion_topic:= <motion_topic> _change_direction_topic:= <change_direction_topic> [Options]\n",
  "Example:  blind_controller_node _odom_topic:= /odom _motion_topic:= /motion _change_direction_topic:= /change_direction \n\n",
  "Options:\n",
  "------------------------------------------\n",
  "_odom_topic:= <string>             topic name of odometry of the platform, default: /odom",
  "_twist_topic:= <string>            topic name of twist to control the platform, default: /cmd_vel",
  "_motion_topic:= <string>           topic name of motion command, default: /motion",
  "_change_direction_topic:= <string> topic name of change_direction command, default: /change_direction",
  "_ack_topic:= <string>              topic name of ack signal, default: /blind_controller_ack",
  "_max_trans_speed:= <float>         maximum translation speed, default 0.2 [m/s]",
  "_max_rot_speed:= <float>           maximum translation speed, default 0.4 [rad/s]",
  "_idle_time:= <float>               idle/sleep time between consecutive actions, default 0.f",
  "-h                                 this help\n",
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
  int c = 1;
  while(c<argc){
    if (! strcmp(argv[c],"-h")){
      printBanner(banner);
      return 1;
    }
    c++;
  }
  
  // set of parameters
  std::string odom_topic; 
  std::string twist_topic;
  std::string motion_topic;
  std::string change_direction_topic;
  std::string ack_topic;
  float idle_time;
  nh.param<std::string>("odom_topic", odom_topic, "/odom");
  nh.param<std::string>("twist_topic", twist_topic, "/blind/cmd_vel");
  nh.param<std::string>("motion_topic", motion_topic, "/motion");
  nh.param<std::string>("change_direction_topic", change_direction_topic, "/change_direction");
  nh.param<std::string>("ack_topic", ack_topic, "/blind_controller_ack");
  nh.param<float>("max_trans_speed", translation_speed, 0.2f);
  nh.param<float>("max_rot_speed", rotational_speed, 0.4f);
  nh.param<float>("idle_time", idle_time, 0.f);

  
  std::cerr << "Using the following params:\n";
  std::cerr << "_odom_topic:=             " << odom_topic << std::endl;
  std::cerr << "_twist_topic:=            " << twist_topic << std::endl;
  std::cerr << "_motion_topic:=           " << motion_topic << std::endl;
  std::cerr << "_change_direction_topic:= " << change_direction_topic << std::endl;
  std::cerr << "_ack_topic:=              " << ack_topic << std::endl;
  std::cerr << "_max_trans_speed:=        " << translation_speed << std::endl;
  std::cerr << "_max_rot_speed:=          " << rotational_speed << std::endl;
  std::cerr << "_idle_time:=              " << idle_time << std::endl;    


  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odometryCallback);
  ros::Subscriber motion_sub = nh.subscribe(motion_topic, 1, motionCallback);
  ros::Subscriber change_dir_sub = nh.subscribe(change_direction_topic, 1, changeDirectionCallback);
  
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(twist_topic, 10);
  ack_pub = new ros::Publisher(nh.advertise<std_msgs::String>(ack_topic, 10));

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
	ack_pub->publish(ack_msg);
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
