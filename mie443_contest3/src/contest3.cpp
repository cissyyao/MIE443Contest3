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
double frontDist = 0.0;
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;

bool bumperLeft=0, bumperCenter=0, bumperRight=0;
bool wheelLeft = 0, wheelRight = 0;
bool timer_on = 0;

double avgranges(double arraytoAvg [], int start, int end) {
    double sum = 0.0;
    double n = 0.0;
    int i;
    
    for (i=start; i<=end; ++i) {

        if (arraytoAvg[i] == arraytoAvg[i]) {		// check if a real number
            sum += arraytoAvg[i];
            n+=1;
        }
    }
    
    return sum/n;
}


void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){ 
	//Fill with code
	if(msg.bumper == 0){
		bumperLeft = !bumperLeft;
		world_state = 1;
		}
	else if(msg.bumper == 1){
		bumperCenter = !bumperCenter;
		world_state = 2; // just for testing, should be 3
		}
	else if(msg.bumper == 2){
		bumperRight = !bumperRight;
		world_state = 0;
		}
}

void wheel_dropCB(const kobuki_msgs::WheelDropEvent msg ) {
	
	//int wheelstate = msg->state;
	
	//if wheel drop sensors lifted (state 1), world_state = 6
	if(msg.DROPPED == 1)	{
		wheelRight = !wheelRight;
		ROS_INFO("RIGHT WHEEL IS LIFTED");
		world_state = 6;
		
		
	}
	
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
		std::cout<<"person found"<<std::endl;
	}
	//std::cout<< poseX <<std::endl;
	//std::cout<< poseY <<std::endl;
	//std::cout<< poseZ <<std::endl;
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
	ros::Timer timer = nh.createTimer(ros::Duration(10), timerCallback);
	
	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	world_state = 0;

	double angular = 0.2;
	double linear = 0.0;
	bool FirstTime = true;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;
	
	bool FirstTimeSurprised = true;



	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................
		
		//check owner's disappearance
		
		ROS_INFO("World State: %d", world_state);
		
		
		//neutral state, following human
		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			
			vel_pub.publish(follow_cmd);
			//show image of happy franklin
			//play happy music
			//if (!bumperCenter && !bumperLeft && !bumperRight && frontDist < 1 && !wheelLeft && !wheelRight){
			//	world_state = 1;
			//}

		}

		//world_state1 is surprised at owner's disappearance
		else if(world_state == 1){
			
			vel_pub.publish(follow_cmd);			
			/*if (follow_cmd > 0.05 && !bumperCenter && !bumperLeft && !bumperRight && frontDist < 1 && !wheelLeft && !wheelRight) {
				world_state = 0;
			}*/
				
			//if (FirstTime == true){
				//FirstTime = false;
				linear = -0.5;
				angular = 0;
				vel.linear.x = linear;
				vel.angular.z = 0;
				vel_pub.publish(vel);
				ros::Duration(1).sleep();
				linear = 0;
				vel.linear.x = linear;
				vel_pub.publish(vel);
				//}
				sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/surpised.wav"); //figure out which the sound is playing continuously
				Mat surpriseFace = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/surpriseFace.jpg");
				namedWindow("surpriseFace", WINDOW_AUTOSIZE);
				imshow("surpriseFace", surpriseFace); 
				waitKey(2000);
				destroyWindow("surpriseFace");
				
				//jerk back in surprise initially
				
				
			
			if (!timer){
				world_state = 0;
			}
			
		}
		//after >15s of owner's disappearance
		else if(world_state == 2){
			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/sound.wav");
			Mat scaredFace = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/scaredFace.jpg");
			namedWindow("scaredFace", WINDOW_AUTOSIZE);
			imshow("scaredFace", scaredFace); 
			waitKey(2000);
			destroyWindow("scaredFace");
			
			angular = pi/2;
			linear = 0;
			vel.linear.x = linear;
			vel.angular.z = 0;
			vel_pub.publish(vel);
			ros::Duration(2).sleep();
			angular = -pi/4;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			ros::Duration(2).sleep();
			vel.angular.z = 0;
			vel_pub.publish(vel);
			
			/*if (vel_pub.publish > 0.05 && !bumperCenter && !bumperLeft && !bumperRight && frontDist < 1 && !wheelLeft && !wheelRight) {
				world_state = 0;
			}*/
		}
		
		//world_state == 3, cannot track bc obstacle
		//world_state=4, bumper hit once
		else if(world_state == 4) {
			Mat Anger = imread("/home/cissy/catkin_ws/src/mie443_contest3/Anger.png", CV_LOAD_IMAGE_COLOR);
			namedWindow("Anger", WINDOW_AUTOSIZE);
			imshow("Anger", Anger); 
			waitKey(2000);
			destroyWindow("Anger");
		//world_state=5, bumper hit multiple times
		}
		else if(world_state == 5) {
			Mat Rage = imread("/home/cissy/catkin_ws/src/mie443_contest3/Rage.jpg", CV_LOAD_IMAGE_COLOR);
			namedWindow("Rage", WINDOW_AUTOSIZE);
			imshow("Rage", Rage); 
			waitKey(2000);
			destroyWindow("Rage");
		}
		//robot lifted up (excited)
		else if(world_state == 6) { 
			//display franklin excited face
			//play laughing sound
			//sc.playWave(mie443_contest3/sounds+"excited.wav"); //figure out which the sound is playing continuously
			
			ROS_INFO("World State = %d", world_state);
			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/sound.wav");
			Mat excitedFace = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/excitedFace.jpg");
			namedWindow("excitedFace", WINDOW_AUTOSIZE);
			imshow("excitedFace", excitedFace); 
			waitKey(2000);
			destroyWindow("excitedFace");
			ros::Duration(5).sleep();
			world_state = 0;
			ROS_INFO("EXCITED!!");
		}
	}

	return 0;
}

