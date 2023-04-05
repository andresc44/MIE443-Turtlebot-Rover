#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;
// ---------- Callback Functions -------------
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
}

// ----------- Fear Function --------------
void fearMode(ros::Publisher vel_pub, ros::Subscriber ) {
	int fear_vel = 0.3;
	int fear_rev_vel = -0.1;
	int fear_forward_time = 1;
	int rev_cnt = 0;
	ros::Rate loop_rate(LOOP_RATE);
	std::chrono::time_point<std::chrono::system_clock> start;
	uint64_t seconds_elapsed = 0;                                    

	while (ros::ok() && !AnyBumperPressed && ){	
		// spin around 
		rotate(180);
		rotate(180);

		// go forwards
		start = std::chrono::system_clock::now();
		while (ros::ok() && seconds_elapsed<=fear_forward_time){
			vel_pub.publish(fear_vel);
			loop_rate.sleep()
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
		}			

		// turn slgihtly and go forwards
		rotate(30);
		start = std::chrono::system_clock::now();
		while (ros::ok() && seconds_elapsed<=0.5){
			vel_pub.publish(fear_vel);
			loop_rate.sleep()
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
		}	
		
		// turn slightly in other direction and go forwards	
		rotate(-60);
		start = std::chrono::system_clock::now();
		while (ros::ok() && seconds_elapsed<=0.5){
			vel_pub.publish(fear_vel);
			loop_rate.sleep()
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
		}	

		// slowly go backwards and express fear sound
		rotate(30);
		start = std::chrono::system_clock::now();
		while (ros::ok() && seconds_elapsed<=0.5 && rev_cnt =3){
			vel_pub.publish(fear_rev_vel);
			loop_rate.sleep()
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
			rev_cnt += 1;
		}	

	}
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

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			/*
			...
			...
			*/
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
