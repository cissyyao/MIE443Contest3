#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>

using namespace std; //hello

geometry_msgs::Twist follow_cmd;
int world_state;

double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){ //need to change this!!!!!
    //Fill with code
}

void cliffCB(const kobuki_msgs::CliffEvent msg ) {
	
	//if cliff sensors at state 1, 	world_state = 6;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);
	laserRange = 11;

	if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min){
		for (int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
			if(laserRange > msg->ranges[i])
				laserRange = msg->ranges[i];
		}
	}
	else{
		for(int i = 0; i < laserSize; i++){
			if (laserRange > msg->ranges[i])
				laserRange = msg->ranges[i];
		}
	}

	if (laserRange == 11)
	laserRange = 0;

	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
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
	ros::Subscriber cliff = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);
	
	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now();
	uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;
	
	bool FirstTimeSurprised = true;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);
			
			if (vel_pub.publish < 0.05 && !frontBump && !leftBump && !rightBump && frontdist > 1){
				world_state = 1;
			}

		}
		//world_state1 is surprised at owner's disappearance
		else if(world_state == 1){
			
			vel_pub.publish(follow_cmd);
			
			if (vel_pub.publish > 0.05 && !frontBump && !leftBump && !rightBump && frontDist < 1){
				world_state = 0;
			}
				
			if (FirstTime == true){
				suprisedStart = secondsElapsed;
				FirstTime = false;
				sc.playWave(mie443_contest3/sounds+"surpised.wav"); //figure out which the sound is playing continuously
				Mat surpriseFace = imread("filename");
				imshow(surpriseFace);
				
				//jerk back in surprise initially
				linear = -2;
				angular = 0;
				vel.linear.x = linear;
				vel.angular.z = 0;
				vel_pub.publish(vel);
				sleep(2);
				linear = 0;
				vel.linear.x = linear;
				vel_pub.publish(vel);
				
				
			}
			
			if ((secondsElapsed - surprisedStart) >= 15){
				world_state = 2;
				FirstTime = true;
			}
			
		}
		else if(world_state == 2){
			sc.playWave(mie443_contest3/sounds+"scared.wav");
			Mat scaredFace = imread("filename");
			imshow(scaredFace);
			
			angular = pi;
			linear = 0;
			vel.linear.x = linear;
			vel.angular.z = 0;
			vel_pub.publish(vel);
			sleep(1);
			angular = -pi;
			vel.angular.z = 0;
			vel_pub.publish(vel);
			sleep(2);
			
			if(			
		}
		
		else if(world_state == 6) { //i.e. lifted up
			//display franklin excited face
			//play laughing sound
			
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}
