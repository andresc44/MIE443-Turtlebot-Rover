#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/BumperEvent.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>

//#include "led_manager/LedAnim.h" // Selin

using namespace std;
#define LOOP_RATE 10                            	// Rate for while loops that dictate callback and publish frequency (Hz)
#define FORWARD_DIST 0.5
#define TURNING_V 0.6 //Adam
#define BACKWARD_V -0.3 //Adam
#define N_BUMPER (3)                            // Number of bumpers on kobuki base
#define RAGE_VEL 0.5
#define RAGE_TURN 1.5
#define REVERSE_TIME 3
#define BACKWARD_T 0.5
#define FEAR_TIME 2.5
#define DEG2RAD(deg) ((deg) * M_PI / 180.)      // Inverse conversion function

// sound_play::SoundClient sc;
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist Vel;							// Create message for velocities as Twist type
ros::Publisher VelPub;

// std::string path_to_sounds ("/home/thursday/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/sounds");
string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";

uint8_t Bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool AnyBumperPressed = false;                  // Reset variable to false
bool FollowerInReverse = false;
bool MovingForward = true;

bool FollowingHuman = true;

int WorldState;


// creating the image files for emotions
cv::Mat sad_image = cv::imread("/home/thurs/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Sad.png",cv::IMREAD_UNCHANGED);
cv::Mat excited_image = cv::imread("/home/thurs/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Excited.png",cv::IMREAD_UNCHANGED);
cv::Mat fear_image = cv::imread("/home/thurs/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Fear.png",cv::IMREAD_UNCHANGED);
cv::Mat rage_image = cv::imread("/home/thurs/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Rage.png",cv::IMREAD_UNCHANGED);
cv::Mat neutral_image = cv::imread("/home/thurs/catkin_ws/src/MIE443-Turtlebot-Rover/mie443_contest3/images/Neutral.png",cv::IMREAD_UNCHANGED);

double scaleFactor = 1.25;

cv::resize(sad_image, sad_image, cv::Size(sad_image.cols*scaleFactor, sad_image.rows*scaleFactor), cv::INTER_LINEAR);
cv::resize(excited_image, excited_image, cv::Size(excited_image.cols*scaleFactor, excited_image.rows*scaleFactor), cv::INTER_LINEAR);
cv::resize(fear_image, fear_image, cv::Size(fear_image.cols*scaleFactor, fear_image.rows*scaleFactor), cv::INTER_LINEAR);
cv::resize(rage_image, rage_image, cv::Size(rage_image.cols*scaleFactor, rage_image.rows*scaleFactor), cv::INTER_LINEAR);
cv::resize(neutral_image, neutral_image, cv::Size(neutral_image.cols*scaleFactor, neutral_image.rows*scaleFactor), cv::INTER_LINEAR);
	
int sign(float number) {
    return (number>0)? 1: -1;                               // Positive -> 1, otherwise -1
}
void personCB (const visualization_msgs::Marker msg) {
	float person_x = msg.pose.position.x;
	float person_y = msg.pose.position.y;
	float person_z = msg.pose.position.z;
	// ROS_INFO("Person x: %f, Person y: %f, Person z: %f", person_x, person_y, person_z);
	if ((person_x != 0) && (person_y != 0)) FollowingHuman = true;
	else FollowingHuman = false;
}

void followerCB(const geometry_msgs::Twist msg){ 	// Andres
    follow_cmd = msg;
	FollowerInReverse = false;
	MovingForward = true;
	if (msg.linear.x < -0.1) FollowerInReverse = true;
	else FollowerInReverse = false;
	if (msg.linear.x > 0.1) MovingForward = true;
	else MovingForward = false;
}

//Adam's branch/////////////////////////////////////
void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    Bumper[msg->bumper] = msg->state;            	// Assigns bumper array to match message based on msg message
    AnyBumperPressed = false;
    for (uint8_t i = 0; i < N_BUMPER; i++){
        if (Bumper[i] == kobuki_msgs::BumperEvent::PRESSED) AnyBumperPressed = true; // bumper[0] = leftState, bumper[1] = centerState, bumper[2] = rightState
    }
    // if (AnyBumperPressed) WorldState = 2; Moved to main
}

void rotate(ros::Publisher vel_pub, int angle_rot){                  // Called with input as an angle in degree
    ROS_INFO("rotating by: %i",angle_rot);

   int dir = sign(angle_rot);
//    ROS_INFO("direction: %i", dir);
   float angle_rad = DEG2RAD(dir*angle_rot);                        // Function takes in an angle in degrees, converts it to rad
   float turning_time = angle_rad/TURNING_V;                        // Calculates the time needed to turn based on constant turning speed, and the angle
       
   std::chrono::time_point<std::chrono::system_clock> start;        // Initialize timer
   start = std::chrono::system_clock::now();                        // Start new timer at current time
   uint64_t seconds_elapsed = 0;                                    // New variable for time that has passed
    
   ros::Rate loop_rate(LOOP_RATE);
   Vel.linear.x = 0;
   Vel.angular.z = dir * TURNING_V;                                 // Set turning speed  
//    ROS_INFO("direction: %f", dir*TURNING_V);
    while (ros::ok() && seconds_elapsed <= turning_time) {          // Turn until the turning time needed to complete the angle is passed   
        vel_pub.publish(Vel);                                        // Start turning
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
}
////////////////////////////////////////////////////////////

void neutralMode(ros::Publisher vel_pub) {
	bool placeholder_for_natural_state = true;
	vel_pub.publish(follow_cmd);
	ROS_INFO("Being Neutral");
}

void fearMode(ros::Publisher vel_pub) {
	float fear_vel = 0.3;
	float fear_rev_vel = -0.1;
	int fear_forward_time = 1;
	int rev_cnt = 0;
	ros::Rate loop_rate(LOOP_RATE);
	std::chrono::time_point<std::chrono::system_clock> start;
	uint64_t seconds_elapsed = 0;                 
	bool FollowingHuman = false;   

	Vel.angular.z = 0.0;
    Vel.linear.x = 0.0;              

	while (ros::ok() && !AnyBumperPressed && !FollowingHuman){	
		// spin around 
		Vel.linear.x = 0.0;
		Vel.angular.z = 0.5;
		vel_pub.publish(Vel);
		rotate(vel_pub, 180);
		rotate(vel_pub, 180);

		// go forwards
		start = std::chrono::system_clock::now();
		ROS_INFO("Fear Forward");
		while (ros::ok() && seconds_elapsed<=fear_forward_time && !FollowingHuman){
			Vel.linear.x = fear_vel;
			Vel.angular.z = 0.0;
			vel_pub.publish(Vel);
			loop_rate.sleep();
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
			ros::spinOnce();
		}			

		// turn slgihtly and go forwards
		Vel.linear.x = 0.0;
		Vel.angular.z = 0.5;
		vel_pub.publish(Vel);
		rotate(vel_pub, 30);
		start = std::chrono::system_clock::now();
		ROS_INFO("Turn slightly and forward");
		while (ros::ok() && seconds_elapsed<=0.5 && !FollowingHuman){
			Vel.linear.x = fear_vel;
			Vel.angular.z = 0.0;
			vel_pub.publish(Vel);
			loop_rate.sleep();
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
			ros::spinOnce();
		}	
		
		// turn slightly in other direction and go forwards	
		Vel.linear.x = 0.0;
		Vel.angular.z = 0.5;
		vel_pub.publish(Vel);
		rotate(vel_pub, -60);
		start = std::chrono::system_clock::now();
		ROS_INFO("Turn slightly other dir. and forward");
		while (ros::ok() && seconds_elapsed<=0.5 && !FollowingHuman){
			Vel.linear.x = fear_vel;
			Vel.angular.z = 0.0;
			vel_pub.publish(Vel);
			loop_rate.sleep();
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
			ros::spinOnce();
		}	

		// slowly go backwards and express fear sound
		Vel.linear.x = 0.0;
		Vel.angular.z = 0.2;
		vel_pub.publish(Vel);
		rotate(vel_pub, 30);
		start = std::chrono::system_clock::now();
		ROS_INFO("backwards and noise");
		while (ros::ok() && seconds_elapsed<=0.5 && rev_cnt == 3 && !FollowingHuman){
			Vel.linear.x = fear_rev_vel;
			Vel.angular.z = 0.0;
			vel_pub.publish(Vel);
			loop_rate.sleep();
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
			rev_cnt += 1;
			ros::spinOnce();
		}	
	}
}

void sadMode(ros::Publisher vel_pub) {
	Vel.linear.x = BACKWARD_V;                                      	// Set linear to Twist
	Vel.angular.z = 0;
	uint64_t seconds_elapsed = 0;                                   // New variable for time that has passed    
	float backward_time = 0.5/abs(BACKWARD_V);   //0.5 m backwards
	ros::Rate loop_rate(LOOP_RATE);
	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now();                       // Start new timer at current time
	ROS_INFO("Going backwards because sad");
	while (ros::ok() && seconds_elapsed <= backward_time) {         // Kobuki diameter is 0.3515m, we want to travel half (3.5s x 0.05m/s) 
		vel_pub.publish(Vel);                                        // Publish cmd_vel to teleop mux input
		loop_rate.sleep();
		seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
		//ROS_INFO("seconds_elapsed:%i,backward_T:%f",seconds_elapsed,BACKWARD_T);
	}
	//////start being sad
	// sc.playWave(path_to_sounds + "sound.wav"); //starts playing crying noises
	
	rotate(vel_pub, -23); //rotate back an fourth while crying 
	//delay()
	rotate(vel_pub,45);
	//delay()
	rotate(vel_pub,-45);
	//delay()
	rotate(vel_pub, 23);
	//delay
	// sc.stopPlayingWave(); //stops the sound CAUSES CATKIN_MAKE ERROR
	WorldState = 0;
}

void excitedMode(ros::Publisher vel_pub) {
	// Move TurtleBot in circles for 10 seconds
	Vel.angular.z = 1.0;
	Vel.linear.x = 0.0;                                      	// Set linear to Twist
	uint64_t seconds_elapsed = 0;                                   // New variable for time that has passed    
	float move_time = 10;   //0.5 m backwards
	ros::Rate loop_rate(LOOP_RATE);
	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now();                       // Start new timer at current time
	while (ros::ok() && seconds_elapsed <= move_time) {         // Kobuki diameter is 0.3515m, we want to travel half (3.5s x 0.05m/s) 
		vel_pub.publish(Vel);                                        // Publish cmd_vel to teleop mux input
		loop_rate.sleep();
		seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
		//ROS_INFO("seconds_elapsed:%i,backward_T:%f",seconds_elapsed,BACKWARD_T);
	}
	
	// Play happy/exciting song for 3 seconds
	// sc.playWave(path_to_sounds + "sound.wav"); //add happy/exciting sound and modify the name
	// sound_play::SoundClient sc;

	// // Blink the LEDs in different colors
	// led_manager::LedAnim led_anim;
	// led_anim.request.name = "blink";
	// led_anim.request.color.r = 255;
	// led_anim.request.color.g = 0;
	// led_anim.request.color.b = 0;
	// led_anim.request.frequency = 5;
	// led_anim.request.duration = 5.0;
	// led_client.call(led_anim);
	WorldState = 0;
}


void rageMode(ros::Publisher vel_pub) {
	// Shake a little
	// Publish forward in a loop until bumper is pressed
	// Make noises
	// Turn screen red 

	// sc.playWave(path_to_sounds + "sound.wav"); //Get out of me swamp
	ros::Duration(0.5).sleep();

	ros::Rate loop_rate(LOOP_RATE);
    std::chrono::time_point<std::chrono::system_clock> start;       // Initialize timer

    Vel.angular.z = 0.0;
    Vel.linear.x = RAGE_VEL;
    uint64_t seconds_elapsed = 0;                                    // Variable for time that has passed

	ROS_INFO("Rushing person");
    while (ros::ok()  && !AnyBumperPressed) {						// Move forward until it hits feet
        vel_pub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        loop_rate.sleep();
    }
	// "get out of my way"
	// sc.playWave(path_to_sounds + "sound.wav"); //Move/get out, consider putting in bumper callback

	ROS_INFO("Foot hit, reversing");

    Vel.linear.x = BACKWARD_V;                                      // Set linear to Twist
    Vel.angular.z = 0;
    seconds_elapsed = 0;                                   // New variable for time that has passed    
    int angle = 0;

    start = std::chrono::system_clock::now();                       // Start new timer at current time
	// Move back a little
    while (ros::ok() && seconds_elapsed <= BACKWARD_T) {            // Kobuki diameter is 0.3515m, we want to travel half (3.5s x 0.05m/s) 
        vel_pub.publish(Vel);                                        // Publish cmd_vel to teleop mux input
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
        //ROS_INFO("seconds_elapsed:%i,backward_T:%f",seconds_elapsed,BACKWARD_T);
    }
	// Turn right
    rotate(VelPub, -90); 

	Vel.angular.z = 0.0;
    Vel.linear.x = RAGE_VEL;
    seconds_elapsed = 0;                                    // Variable for time that has passed
	float time_forward = abs(FORWARD_DIST/RAGE_VEL);

    start = std::chrono::system_clock::now();                       // Start the timer
	ros::spinOnce();
	ROS_INFO("Move away from victim");
    while (ros::ok() && seconds_elapsed <= time_forward && !AnyBumperPressed) { //Move forward a few steps
        vel_pub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }

	// sc.playWave(path_to_sounds + "sound.wav"); //Rooooooaaaarrrr
	Vel.angular.z = RAGE_TURN;
    Vel.linear.x = 0.0;
    seconds_elapsed = 0;                                    			// Variable for time that has passed
	float time_turning = 3.0;

    start = std::chrono::system_clock::now();                       // Start the timer
	ros::spinOnce();
	ROS_INFO("Angry spin");
    while (ros::ok() && seconds_elapsed <= time_turning) { // Spin for 3 seconds
        vel_pub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
}


//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emotional_follower");
	ros::NodeHandle nh;
	
	sound_play::SoundClient sc;
	// string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	// string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	VelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
 	//ros::ServiceClient led_client = nh.serviceClient<led_manager::LedAnim>("led_anim");

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber person = nh.subscribe("turtlebot_follower/marker", 10, &personCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
	std::chrono::time_point<std::chrono::system_clock> reverse_start;
    std::chrono::time_point<std::chrono::system_clock> fear_start;


	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	// geometry_msgs::Twist vel; //unnecessary, likely will be deleted
	// vel.angular.z = 0.2;
	// vel.linear.x = 0.0;
	sc.playWave(path_to_sounds + "sound.wav");
	
	start = std::chrono::system_clock::now();
	reverse_start = start;
	fear_start = start;
	// buffer_start = std::chrono::system_clock::now(); May cause issues
	// reverse_start = std::chrono::system_clock::now();

	
	WorldState = 0;
    uint64_t seconds_elapsed = 0;
	uint64_t time_in_reverse = 0;
	uint64_t time_alone = 0;
	uint8_t last_state = 0;
	bool is_alone = false;
	bool is_reversing = false;
	ros::spinOnce();
	ros::Duration(0.5).sleep();
	ROS_INFO("Entering while loop");
	while(ros::ok() && seconds_elapsed <= 480){		
		ros::spinOnce();
		ROS_INFO("FollowingHuman: %i, AnyBumperPressed: %i, MovingForward: %i, FollowerInReverse: %i", FollowingHuman, AnyBumperPressed, MovingForward, FollowerInReverse);

// // try {
// //         if(isValid) {
// //             img.release(); //previous image is released from memory 
// //         }
// //         img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
// //         isValid = true;
// //     } catch (cv_bridge::Exception& e) {
// //         std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
// //                   << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
// //         isValid = false;
// //     }    
// // }


		time_in_reverse = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-reverse_start).count();
		time_alone = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-fear_start).count();
		

		if (!is_alone && !FollowingHuman){
			fear_start = std::chrono::system_clock::now();
			time_alone = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-fear_start).count();
			is_alone = true;
		}
		else if (FollowingHuman) is_alone = false;

		if (!is_reversing && FollowerInReverse){
			reverse_start = std::chrono::system_clock::now();
			time_in_reverse = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-reverse_start).count();
			is_reversing = true;
		}
		else if (MovingForward) is_reversing = false;
		ROS_INFO("last_state: %i, time: %i, reverse time: %i, alone time: %i, reversing?: %i, alone?: %i", last_state, seconds_elapsed, time_in_reverse, time_alone, is_reversing, is_alone);
		// Fear Mode Conditions
		if ((is_alone) && (time_alone > FEAR_TIME) && (last_state != 3) && (last_state != 4)) WorldState = 1;

		// Sad Mode Conditions
		else if (AnyBumperPressed) WorldState = 2;

		// Excitement Mode Conditions
		else if ((FollowingHuman) && (last_state == 1)) WorldState = 3;

		// Rage Mode Conditions
		else if (is_reversing && (time_in_reverse > REVERSE_TIME)) WorldState = 4;

		else WorldState = 0;

		ROS_INFO("Worldstate: ", WorldState);
		switch (WorldState) {
			case 0:
				neutralMode(VelPub); //idk play music or show images or something
				ROS_INFO("Following Human as normal");
				break;
			case 1:
				sc.playWave(path_to_sounds + "fear.wav");
				cv::destroyAllWindows();
				cv::imshow("view", fear_image);
				ROS_INFO("Entering fear mode");
				fearMode(VelPub);
				ROS_INFO("Broke fear mode");
				last_state = 1;
				is_alone = false;
				cv::destroyAllWindows();
				cv::imshow("view", neutral_image);
				break;
			case 2:
				sc.playWave(path_to_sounds + "sad.wav");
				cv::destroyAllWindows();
				cv::imshow("view", sad_image);
				ROS_INFO("Hit something, entering sad mode");
				sadMode(VelPub);
				ROS_INFO("Broke sad mode");
				last_state = 2;
				cv::destroyAllWindows();
				cv::imshow("view", neutral_image);
				break;
			case 3:
				sc.playWave(path_to_sounds + "excited.wav");
				cv::destroyAllWindows();
				cv::imshow("view", excited_image);
				ROS_INFO("Found my person, getting excited");
				excitedMode(VelPub);
				ROS_INFO("Breaking excited mode");
				last_state = 3;
				cv::destroyAllWindows();
				cv::imshow("view", neutral_image);
				break;
			case 4:
				sc.playWave(path_to_sounds + "rage.wav");
				cv::destroyAllWindows();
				cv::imshow("view", rage_image);
				ROS_INFO("Got intimidated, getting angry");
				rageMode(VelPub);
				ROS_INFO("Breaking rage mode");
				is_reversing = false;
				last_state = 4;
				cv::destroyAllWindows();
				cv::imshow("view", neutral_image);
				break;
		}

		seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}
	ROS_INFO("Main while loop ended");
	//Play "I'm a Believer" and flash the lights

	return 0;
}
