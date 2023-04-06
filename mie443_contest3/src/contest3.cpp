#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
//#include "led_manager/LedAnim.h"

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;

bool state1 = true;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
  	//ros::ServiceClient led_client = nh.serviceClient<led_manager::LedAnim>("led_anim");

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

    // contest count down timer
	ros::Rate loop_rate(10);
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

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	// creating the image files for emotions
	cv::Mat sad_image = cv::imread("/home/selin/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Sad.png",cv::IMREAD_UNCHANGED);
	cv::Mat excited_image = cv::imread("/home/selin/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Excited.png",cv::IMREAD_UNCHANGED);
	cv::Mat fear_image = cv::imread("/home/selin/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Fear.png",cv::IMREAD_UNCHANGED);
	cv::Mat rage_image = cv::imread("/home/selin/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Rage.png",cv::IMREAD_UNCHANGED);
	cv::Mat neutral_image = cv::imread("/home/selin/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Neutral.png",cv::IMREAD_UNCHANGED);

	double scaleFactor = 1.25;

	cv::resize(sad_image, sad_image, cv::Size(sad_image.cols*scaleFactor, sad_image.rows*scaleFactor), cv::INTER_LINEAR);
	cv::resize(excited_image, excited_image, cv::Size(excited_image.cols*scaleFactor, excited_image.rows*scaleFactor), cv::INTER_LINEAR);
	cv::resize(fear_image, fear_image, cv::Size(fear_image.cols*scaleFactor, fear_image.rows*scaleFactor), cv::INTER_LINEAR);
	cv::resize(rage_image, rage_image, cv::Size(rage_image.cols*scaleFactor, rage_image.rows*scaleFactor), cv::INTER_LINEAR);
	cv::resize(neutral_image, neutral_image, cv::Size(neutral_image.cols*scaleFactor, neutral_image.rows*scaleFactor), cv::INTER_LINEAR);

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		//To do for images: 1)make all png 2)make all same size 3)include opencv libraries 4)create images folder
		
		//neutral
		cv::imshow("view", neutral_image);
		ros::Time start_time0 = ros::Time::now(); // Get the current time
		while ((ros::Time::now() - start_time0).toSec() < 5.0) { // Wait for 5 seconds
			cv::waitKey(1); // Wait for 1 millisecond for a key event
		}
		cv::destroyAllWindows();
		ros::Duration(2.0).sleep();		

		//sad
		cv::imshow("view", sad_image);
		ros::Time start_time1 = ros::Time::now(); // Get the current time
		while ((ros::Time::now() - start_time1).toSec() < 5.0) { // Wait for 5 seconds
			cv::waitKey(1); // Wait for 1 millisecond for a key event
		}
		cv::destroyAllWindows();
		ros::Duration(2.0).sleep();

		//excited
		cv::imshow("view", excited_image);
		ros::Time start_time2 = ros::Time::now(); // Get the current time
		while ((ros::Time::now() - start_time2).toSec() < 5.0) { // Wait for 5 seconds
			cv::waitKey(1); // Wait for 1 millisecond for a key event
		}
		cv::destroyAllWindows();
		ros::Duration(2.0).sleep();

		//fear
		cv::imshow("view", fear_image);
		ros::Time start_time3 = ros::Time::now(); // Get the current time
		while ((ros::Time::now() - start_time3).toSec() < 5.0) { // Wait for 5 seconds
			cv::waitKey(1); // Wait for 1 millisecond for a key event
		}
		cv::destroyAllWindows();
		ros::Duration(2.0).sleep();

		//rage
		cv::imshow("view", rage_image);
		ros::Time start_time4 = ros::Time::now(); // Get the current time
		while ((ros::Time::now() - start_time4).toSec() < 5.0) { // Wait for 5 seconds
			cv::waitKey(1); // Wait for 1 millisecond for a key event
		}
		cv::destroyAllWindows();
		ros::Duration(2.0).sleep();

		}
		
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();

	return 0;
}
