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

using namespace std; //hello
using namespace cv; 

geometry_msgs::Twist follow_cmd;
int world_state;

double pi = 3.14159;
double frontDist = 0.0;
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
int bumperCount = 0;

bool bumperLeft=0, bumperCenter=0, bumperRight=0;
bool wheelLeft = 0, wheelRight = 0;

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
	if(bumperCount >=5){
		world_state = 5; //rage
	}
	else if(msg.bumper == 0 || msg.bumper == 2){
		bumperLeft = !bumperLeft;
		bumperRight = !bumperRight;
		world_state = 4; //anger
	}
	else if(msg.bumper == 1){
		bumperCenter = !bumperCenter;
		world_state = 2; // just for testing, should be 3
	}
}

void wheel_dropCB(const kobuki_msgs::WheelDropEvent msg ) {
	
	//if wheel drop sensors lifted (state 1), world_state = 6
	if(msg.DROPPED == 0) {			
		wheelLeft = !wheelLeft;     
		ROS_INFO("LEFT WHEEL IS LIFTED");
		world_state = 0;
	}
	
	else{
	//(msg.DROPPED == 1)	{
		wheelRight = !wheelRight;
		ROS_INFO("RIGHT WHEEL IS LIFTED");
		world_state = 6;
	}
	
}

void timerCallback(const ros::TimerEvent&){ 
	ROS_INFO("Timer Callback triggered");
	world_state = 2;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);

	double x[640], y[640], d[640]; //an array of all x and y distance values of laser readings
	
	double incr= msg->angle_increment;
	for (int i = 0; i<= 639; ++i) {
		d[i] = msg->ranges[i];
		
	}
	std::cout<<"1"<<std::endl;

	frontDist = avgranges(d, 269, 369);
}

/*void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);

	double d[640]; //an array of all d
	
	
	for (int i = 0; i<= 639; ++i) {
		d[i] = msg->ranges[i];
		
	}

	double l= msg ->ranges[300];
	ROS_INFO("%d LAZER",l);
	double n = 0;
	
	//centreD = avgranges(d, 269, 369);
	
	for ( int j=269; j<=369; ++j) {

	        if (d[j] == d[j]) {		// check if a real number
        	    frontDist += d[j];
            	    n+=1;
        	}
}    
   frontDist = frontDist/n;
}*/
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
	ros::Subscriber laser_sub = nh.subscribe("/scan", 10, &laserCallback);
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber wheel_drop = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheel_dropCB);
	

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	world_state = 0;

	double angular = 0.2;
	double linear = 0.0;
	bool FirstTime = true;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;
	
	bool FirstTimeSurprised = true;

	//sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();
	
	

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................
		
		//check owner's disappearance
		/*if (vel_pub.publish < 0.05 && !bumperCenter && !bumperLeft && !bumperRight && frontDist > 1 && !wheelLeft && !wheelRight) { 
		world_state = 1;
		}*/
		ROS_INFO("World State: %d.    FrontDist = %d", world_state, frontDist);
		
		if (!bumperCenter && !bumperLeft && !bumperRight && frontDist < 1 && !wheelLeft && !wheelRight) {
		//		world_state = 0;
	}
		
		//neutral state, following human
		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			
			vel_pub.publish(follow_cmd);
			//show image of happy franklin
			//play happy music
			if (!bumperCenter && !bumperLeft && !bumperRight && frontDist < 1 && !wheelLeft && !wheelRight){
				world_state = 1;
			}

		}

		//world_state1 is surprised at owner's disappearance
		else if(world_state == 1){
			
			vel_pub.publish(follow_cmd);
			ros::Timer timer = nh.createTimer(ros::Duration(15), timerCallback);
			
			/*if (follow_cmd > 0.05 && !bumperCenter && !bumperLeft && !bumperRight && frontDist < 1 && !wheelLeft && !wheelRight) {
				world_state = 0;
			}*/
				
			//if (FirstTime == true){
				//FirstTime = false;
				sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/surpised.wav"); //figure out which the sound is playing continuously
				Mat surpriseFace = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/surpriseFace.jpg");
				namedWindow("surpriseFace", WINDOW_AUTOSIZE);
				imshow("surpriseFace", surpriseFace); 
				waitKey(2000);
				destroyWindow("surpriseFace");
				
				//jerk back in surprise initially
				linear = -2;
				angular = 0;
				vel.linear.x = linear;
				vel.angular.z = 0;
				vel_pub.publish(vel);
				ros::Duration(2).sleep();
				linear = 0;
				vel.linear.x = linear;
				vel_pub.publish(vel);
				
				
			//}
			
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
			
			angular = pi/4;
			linear = 0;
			vel.linear.x = linear;
			vel.angular.z = 0;
			vel_pub.publish(vel);
			ros::Duration(2).sleep();
			angular = -pi/4;
			vel.angular.z = 0;
			vel_pub.publish(vel);
			sleep(2);
			
			/*if (vel_pub.publish > 0.05 && !bumperCenter && !bumperLeft && !bumperRight && frontDist < 1 && !wheelLeft && !wheelRight) {
				world_state = 0;
			}*/
		}
		
		//world_state == 3, cannot track bc obstacle
		//world_state=4, bumper hit once
		else if(world_state == 4) {
			Mat angry = imread("/home/cissy/catkin_ws/src/mie443_contest3/angry.jpg", CV_LOAD_IMAGE_COLOR);
			namedWindow("Angry", WINDOW_AUTOSIZE);
			imshow("Angry", angry); 
			waitKey(2000);
			destroyWindow("Angry");
		//world_state=5, bumper hit multiple times
		}
		else if(world_state == 5) {
			Mat rage = imread("/home/cissy/catkin_ws/src/mie443_contest3/rage.jpg", CV_LOAD_IMAGE_COLOR);
			namedWindow("rage", WINDOW_AUTOSIZE);
			imshow("rage", rage); 
			waitKey(2000);
			destroyWindow("rage");
		}
		//robot lifted up (excited)
		else if(world_state == 6) { 
			//display franklin excited face
			//play laughing sound
			//sc.playWave(mie443_contest3/sounds+"excited.wav"); //figure out which the sound is playing continuously
			Mat excitedFace = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/excitedFace.jpg");
			namedWindow("excitedFace", WINDOW_AUTOSIZE);
			imshow("excitedFace", excitedFace); 
			waitKey(2000);
			destroyWindow("excitedFace");
			ROS_INFO("EXCITED!!");
		}
	}

	return 0;
}

