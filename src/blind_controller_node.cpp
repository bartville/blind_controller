#include <ros/ros.h>
#include <iostream>
#include <blind_controller/motion.h>
#include <blind_controller/change_direction.h>
#include <geometry_msgs/Twist.h>

typedef std::pair<geometry_msgs::Twist, float> TwistTimePair;
typedef std::vector<TwistTimePair> TwistTimePairVector;
// pair of twist_msgs and needed execution time
TwistTimePairVector twist_time_vector;

float translation_speed = 0.1; // 0.1 meter/second
float rotational_speed = 0.1; // 0.1 rad/second

void motionCallback(const blind_controller::motion::ConstPtr &msg){
  std::cerr << "[motionCallback]: received message "<< std::endl;
  std::string direction = msg->direction;
  float meter = msg->meter;
  std::cerr << "dir: " << direction << "  of " << meter << " meters" << std::endl;

  TwistTimePair new_pair;
  geometry_msgs::Twist new_twist;
  float needed_time = fabs(meter)/translation_speed;
  if(!direction.compare("forward")){
    new_twist.linear.x = translation_speed;
  }
  else if(!direction.compare("backward")){
    new_twist.linear.x = -translation_speed;
  }
  else{
    std::cerr << "wrong direction! This message will be destroyed!" << std::endl;
    return;
  }
  new_pair.first = new_twist;
  new_pair.second = needed_time;
  twist_time_vector.push_back(new_pair);
  
}

void changeDirectionCallback(const blind_controller::change_direction::ConstPtr &msg){
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "BlindControllerNode");
  ros::NodeHandle nh("~");

  std::string cmd_vel = "/cmd_vel";
  std::string motion_topic = "/motion";
  std::string change_direction_topic = "/change_direction";
  
  ros::Subscriber motion_sub = nh.subscribe(motion_topic, 10, motionCallback);
  ros::Subscriber change_dir_sub = nh.subscribe(change_direction_topic, 10, changeDirectionCallback);
  
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel, 10);
  ros::Rate loop_rate(30);

  enum State{IDLE, EXECUTING};

  twist_time_vector.clear();
  State current_state = State::IDLE;

  geometry_msgs::Twist stop_msg; //an empty msg used to stop the robot
  
  TwistTimePair current_motion;
  ros::Time current_motion_starting_time;
  
  while(ros::ok()){

    // if the robot is in Executing mode,
    // check for the expired time
    if(current_state == State::EXECUTING) {
      float expired_time = (ros::Time::now() - current_motion_starting_time).toSec(); //check expired time
      if(expired_time > current_motion.second){   // if time is elapsed
	vel_pub.publish(stop_msg);                // stop the robot
	current_state = State::IDLE;              // change state to IDLE
      }else{                                      // otherwise
	vel_pub.publish(current_motion.first);    // continue moving
      }      
    }

    
    // if the robot is in Idle, check for other
    // motions in the twist vector
    if(current_state == State::IDLE) {
      if(twist_time_vector.size()){	                    // if there are new motions
	current_motion = twist_time_vector.front();         // pop first element
	twist_time_vector.erase(twist_time_vector.begin()); // erase first element
	current_state = State::EXECUTING;                   // change state to EXECUTING
	current_motion_starting_time = ros::Time::now();    // reset timer
      }
      vel_pub.publish(stop_msg);
    }   
    
    ros::spinOnce();
    loop_rate.sleep();
    
  }

  return 0;
}
