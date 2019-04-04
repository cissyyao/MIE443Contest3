#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>

using namespace std; //hello

geometry_msgs::Twist follow_cmd;
int world_state;

double frontDist = 0.0;
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
bool bumperLeft=0, bumperCenter=0, bumperRight=0;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){ 
	//Fill with code
	if(msg.bumper == 0)
		bumperLeft = !bumperLeft;
	else if(msg.bumper == 1)
		bumperCenter = !bumperCenter;
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;
}

void cliffCB(const kobuki_msgs::CliffEvent msg ) {
	
	//if cliff sensors at state 1, 	world_state = 6;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);

	double x[640], y[640], d[640]; //an array of all x and y distance values of laser readings
	
	incr= msg->angle_increment;
	angle_ri = msg->angle_min;
	angle_lef= msg->angle_max;
	for (int i = 0; i<= 639; ++i) {
		d[i] = msg->ranges[i];
		x[i] = d[i]*(cos((pi/2.0) - 0.506145 + (i)*incr));
		y[i] = d[i]*(sin((pi/2.0) - 0.506145 + (i)*incr));
		
	}


	if (laserRange == 11)
	laserRange = 0;
	
	double x[640], y[640], d[640];
	incr= msg->angle_increment;
	angle_ri = msg->angle_min;
	angle_lef= msg->angle_max;
	for (int i = 0; i<= 639; ++i) {
		d[i] = msg->ranges[i];
		x[i] = d[i]*(cos((pi/2.0) - 0.506145 + (i)*incr));
		y[i] = d[i]*(sin((pi/2.0) - 0.506145 + (i)*incr));
		
	}
	
	double n = 0.0;
	
	//centreD = avgranges(d, 269, 369);
	
	for (i=269; i<=369; ++i) {

	        if (d[i] == d[i]) {		// check if a real number
        	    frontDist += d[i];
            	    n+=1;
        	}
    }
    
   frontDist = frontDist/n;

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
	bool FirstTime = True;

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
			
			if (vel_pub.publish < 0.05 && !bumperCenter && !bumperLeft && !bumperRight && frontDist > 1){ 
				world_state = 1;
			}

		}
		//world_state1 is surprised at owner's disappearance
		else if(world_state == 1){
			
			vel_pub.publish(follow_cmd);
			
			if (vel_pub.publish > 0.05 && !bumperCenter && !bumperLeft && !bumperRight && frontDist < 1){
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
			else{
				world_state = 0;
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
			
			if (vel_pub.publish > 0.05 && !bumperCenter && !bumperLeft && !bumperRight && frontDist < 1){
				world_state = 0;
			}
		}
		
		else if(world_state == 6) { //i.e. lifted up
			//display franklin excited face
			//play laughing sound
			
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}
