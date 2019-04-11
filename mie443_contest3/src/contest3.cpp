#include <header.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <visualization_msgs/Marker.h>

using namespace std; //hello
using namespace cv; 

geometry_msgs::Twist follow_cmd;
int world_state;

double pi = 3.14159;

bool bumperLeft=0, bumperCenter=0, bumperRight=0;
bool wheelLeft = 0, wheelRight = 0;
bool timer_on = 0;
bool firstTimeSurprised = true;
int bumper_count = 0;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){ 
	//Fill with code
	if(msg.bumper == 0){
		bumperLeft = !bumperLeft;
		bumper_count +=1;
		if (bumper_count < 5){
			world_state = 4;
		}
		else{
			world_state = 5;
		}
	}
	else if(msg.bumper == 1){
		bumperCenter = !bumperCenter;
		world_state = 3;
	}
	else if(msg.bumper == 2){
		bumperRight = !bumperRight;
		bumper_count +=1;
		if (bumper_count < 5){
			world_state = 4;
		}
		else{
			world_state = 5;
		}
	}
	ROS_INFO("bumper_count %d", bumper_count);
}

void wheel_dropCB(const kobuki_msgs::WheelDropEvent msg ) {
	//if wheel drop sensors lifted (state 1), world_state = 6
	if(msg.DROPPED == 1)	{
		wheelRight = !wheelRight;
		ROS_INFO("RIGHT WHEEL IS LIFTED");
		world_state = 6;
	}
	/*
	if(msg.RAISED == 1){ //alternative to the delay in world_state 6 in main
		ROS_INFO("Returned to ground");
		world_state = 0;
	}	
	*/
}

void timerCallback(const ros::TimerEvent&){ 
	ROS_INFO("Timer Callback triggered");
	if (world_state == 1){
		if (timer_on == 0){
			timer_on = 1;
		}
		else{
			world_state = 2;
			timer_on = 0;
		}
	}
	else {
		timer_on = 0;
	}
}

void markerCB (const visualization_msgs::Marker::ConstPtr& msg){
	double poseX = msg->pose.position.x;
	double poseY = msg->pose.position.y;
	double poseZ = msg->pose.position.z;
	
	if (poseX == 0.0 && poseY == 0.0 && poseZ == 1000000.0){
		world_state = 1;
		std::cout<< "Person not in front" <<std::endl;	
	}
	
	else {
		world_state = 0;
		firstTimeSurprised = true;
		std::cout<<"person found"<<std::endl;
	}
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc; //need to rosrun the sound code (separately?)
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber wheel_drop = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheel_dropCB);
	ros::Subscriber marker = nh.subscribe("/turtlebot_follower/marker",10,&markerCB);
	
	ros::Timer timer = nh.createTimer(ros::Duration(7.5), timerCallback);
	
	//imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................
		
		//check owner's disappearance
		
		ROS_INFO("World State: %d", world_state);
		
		if(world_state == 0){ //neutral state, following human
			vel_pub.publish(follow_cmd);
		}
		else if(world_state == 1){ //surprised, cannot find human			
			if (firstTimeSurprised == true){
			
				linear = -0.5;
				angular = 0;
				vel.linear.x = linear;
				vel.angular.z = angular;
				vel_pub.publish(vel);
				ros::Duration(1).sleep();
				
				vel.linear.x = 0;
				vel_pub.publish(vel);
				
				firstTimeSurprised = false;
			}
			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/surprised.wav");
			
			Mat surprised = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/surprised.jpg");
			namedWindow("surprised", WINDOW_AUTOSIZE);
			imshow("surprised", surprised); 
			waitKey(2000);
			destroyWindow("surprised");
		}
		else if(world_state == 2){ //scared, cannot find human after 15s
			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/scared.wav");
			
			Mat scared = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/scared.jpg");
			namedWindow("scared", WINDOW_AUTOSIZE);
			imshow("scared", scared); 
			waitKey(2000);
			destroyWindow("scared");
			
			linear = 0;
			angular = 2*pi;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			ros::Duration(0.25).sleep();
			
			vel.angular.z = 0;
			vel_pub.publish(vel);
			
			angular = -2*pi;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			ros::Duration(0.25).sleep();
			
			vel.angular.z = 0;
			vel_pub.publish(vel);
		}
		else if(world_state == 3) { //sad, cannot follow human (middle bumper hit)
			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/sad.wav");
			
			Mat sad = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/sad.jpg");
			namedWindow("sad", WINDOW_AUTOSIZE);
			imshow("sad", sad); 
			waitKey(2000);
			destroyWindow("sad");
						
			linear = 0;
			angular = pi/4;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			ros::Duration(1).sleep();
			
			vel.angular.z = 0;
			vel_pub.publish(vel);
			
			angular = -pi/4;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			ros::Duration(1).sleep();
			
			vel.angular.z = 0;
			vel_pub.publish(vel);
			
			world_state = 0;
		}
		else if(world_state == 4) { //anger, side bumper hit
			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/anger.wav");
			
			Mat anger = imread("/home/cissy/catkin_ws/src/mie443_contest3/anger.jpg", CV_LOAD_IMAGE_COLOR);
			namedWindow("anger", WINDOW_AUTOSIZE);
			imshow("anger", anger); 
			waitKey(2000);
			destroyWindow("anger");
			
			world_state = 0;
		}
		else if(world_state == 5) { //rage, side bumper hit 5 times
			Mat rage = imread("/home/cissy/catkin_ws/src/mie443_contest3/rage.jpg", CV_LOAD_IMAGE_COLOR);
			namedWindow("rage", WINDOW_AUTOSIZE);
			imshow("rage", rage); 
			waitKey(2000);
			destroyWindow("rage");
			
			linear = 0;
			angular = pi;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			
			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/rage.wav");
			ros::Duration(4).sleep();
			
			vel.angular.z = 0;
			vel_pub.publish(vel);
			
			world_state = 0;
		}
		else if(world_state == 6) { //positively excited, picked up		
			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/excited.wav");
			
			Mat excited = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/excited.jpg");
			namedWindow("excited", WINDOW_AUTOSIZE);
			imshow("excited", excited); 
			waitKey(2000);
			destroyWindow("excited");
			
			ros::Duration(5).sleep();
			world_state = 0;
		}
	}

	return 0;
}

