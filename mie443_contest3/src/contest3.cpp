#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/BumperEvent.h>
//#include "led_manager/LedAnim.h" // Selin

using namespace std;
#define LOOP_RATE 10                            	// Rate for while loops that dictate callback and publish frequency (Hz)
#define FORWARD_DIST 0.5
#define TURNING_V 0.6 //Adam
#define BACKWARD_V -0.1 //Adam
#define N_BUMPER (3)                            // Number of bumpers on kobuki base
#define RAGE_VEL 0.5
#define RAGE_TURN 0.4
#define REVERSE_TIME 8

sound_play::SoundClient sc;
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist Vel;							// Create message for velocities as Twist type
ros::Publisher VelPub;

uint8_t Bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool AnyBumperPressed = false;                  // Reset variable to false
bool FollowerInReverse = false;
bool MovingForward = true;

int WorldState;

void followerCB(const geometry_msgs::Twist msg){ 	//Andres
    follow_cmd = msg;
	FollowerInReverse = false;
	if (msg.linear.x < -0.1) FollowerInReverse = true;
	if (msg.linear.x > 0.1) MovingForward = true;
}

//Adam's branch/////////////////////////////////////
void bumperCB(const geometry_msgs::Twist msg){
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
    
   ros::Rate loop_rate(100);
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

// COPIED FROM CONTEST 1, likely will be deleted //////////////////////////////////////////////////////////////////////////////////////
uint8_t PressedBumper[3]={0,0,0};               // 3 bit array of bumper status

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    Bumper[msg->bumper] = msg->state;                           // Assigns bumper array to match message based on msg message
    AnyBumperPressed = false;
    for (uint8_t i = 0; i < N_BUMPER; i++){
        if (Bumper[i] == kobuki_msgs::BumperEvent::PRESSED){
            // bumper[0] = leftState, bumper[1] = centerState, bumper[2] = rightState
            AnyBumperPressed = true;
            PressedBumper[i] = 1;
        }
        else PressedBumper[i] =0;
    }
    if (AnyBumperPressed) reverse(VelPub);
}

void reverse(ros::Publisher VelPub) {                               // Called if any bumper pressed
    ROS_INFO("Object hit, reversing");
    ros::Rate loop_rate(LOOP_RATE);
    std::chrono::time_point<std::chrono::system_clock> start;       // Initialize timer

    Vel.linear.x = BACKWARD_V;                                      // Set linear to Twist
    Vel.angular.z = 0;
    uint64_t seconds_elapsed = 0;                                   // New variable for time that has passed    
    int angle = 0;

    start = std::chrono::system_clock::now();                       // Start new timer at current time

    while (ros::ok() && seconds_elapsed <= BACKWARD_T) {            // Kobuki diameter is 0.3515m, we want to travel half (3.5s x 0.05m/s) 
        VelPub.publish(Vel);                                        // Publish cmd_vel to teleop mux input
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
        //ROS_INFO("seconds_elapsed:%i,backward_T:%f",seconds_elapsed,BACKWARD_T);
    }
    
    if (arrayEqual(PressedBumper,LState)) angle = -45;            // Left bumper, rotate right (right is neg)
    
    else if (arrayEqual(PressedBumper, LMState)) angle = -90;      // Left and middle pressed only

    else if (arrayEqual(PressedBumper,RState)) angle = 45;        // Right pressed only, rotate left (pos)

    else if (arrayEqual(PressedBumper,RMState)) angle = 90;       // Right and middle pressed only, rotate left

    else if (arrayEqual(PressedBumper,MState)) {                  // Middle pressed only
        // Rotate random angle
        int array_angles[2] = {90, -90};                            // Array of random angles
        int rand_num = rand() % 2;                                  // Random number from 0 to 1
        angle = array_angles[rand_num];                             // Index the array angles using random number, to rotate either left or right 90 degrees
    }

    else if  (arrayEqual(PressedBumper,LMRState) || arrayEqual(PressedBumper,LRState)) { // All pressed or left and right pressed
       // Rotate random angle
        int array_angles[2] = {135, -135};                          // Array of random angles
        int rand_num = rand() % 2;                                  // Random number from 0 to 1
        angle = array_angles[rand_num];                             // Index the array angles using random number, to rotate either left or right 135 degrees 
    }
   
    rotate(VelPub, angle);                                          // Rotate function
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void fearMode (ros::Publisher vel_pub) {
	bool placeholder_for_Maggies_code = true;
	// need some sort of break to stop erratic behaviour, after seeing a person from ros spinOnce();
}

void sadMode(ros::Publisher vel_pub) {
	Vel.linear.x = BACKWARD_V;                                      	// Set linear to Twist
	Vel.angular.z = 0;
		uint64_t seconds_elapsed = 0;                                   // New variable for time that has passed    
	Backward_time = 0.5/abs(BACKWARD_V);   //0.5 m backwards
		start = std::chrono::system_clock::now();                       // Start new timer at current time

		while (ros::ok() && seconds_elapsed <= Backward_time) {         // Kobuki diameter is 0.3515m, we want to travel half (3.5s x 0.05m/s) 
			vel_pub.publish(Vel);                                        // Publish cmd_vel to teleop mux input
			loop_rate.sleep();
			seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
			//ROS_INFO("seconds_elapsed:%i,backward_T:%f",seconds_elapsed,BACKWARD_T);
		}
	//////start being sad
	sc.playWave(path_to_sounds + "sound.wav"); //starts playing crying noises
	
	rotate(vel_pub, -45); //rotate back an fourth while crying 
	//delay()
	rotate(vel_pub,45);
	//delay()
	rotate(vel_pub,-45);
	//delay()
	rotate(vel_pub, 45);
	//delay
	sc.stopPlayingWave(); //stops the sound
	WorldState = 0;
}

void excitedMode(ros::Publisher vel_pub) {
	// Move TurtleBot in circles for 10 seconds
	Vel.angular.z = 1.0;
	for (int i=0; i < 10; i++){ 
		vel_pub.publish(vel);
		ros::Duration(1.0).sleep();
	}
	
	// Play happy/exciting song for 3 seconds
	sc.playWave(path_to_sounds + "sound.wav"); //add happy/exciting sound and modify the name
	ros::Duration(3).sleep();

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

	sc.playWave(path_to_sounds + "sound.wav"); //Get out of me swamp
	ros::Duration(0.5).sleep();

	ros::Rate loop_rate(LOOP_RATE);
    std::chrono::time_point<std::chrono::system_clock> start;       // Initialize timer

    Vel.angular.z = 0.0;
    Vel.linear.x = RAGE_VEL;
    uint64_t seconds_elapsed = 0;                                    // Variable for time that has passed


    while (ros::ok()  && !AnyBumperPressed) {
        vel_pub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        if (AnyBumperPressed) {
            break;
        }
        loop_rate.sleep();
    }
	// "get out of my way"
	sc.playWave(path_to_sounds + "sound.wav"); //Move/get out, consider putting in bumper callback
	ros::Duration(0.5).sleep();

	Vel.angular.z = 0.0;
    Vel.linear.x = RAGE_VEL;
    seconds_elapsed = 0;                                    // Variable for time that has passed
	float time_forward = abs(FORWARD_DIST/RAGE_VEL);

    start = std::chrono::system_clock::now();                       // Start the timer
	ros::spinOnce();

    while (ros::ok() && seconds_elapsed <= time_forward && !AnyBumperPressed) { //Move forward
        vel_pub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        // if (AnyBumperPressed) {
        //     break;
        // }
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }

	Vel.angular.z = RAGE_TURN;
    Vel.linear.x = 0.0;
    seconds_elapsed = 0;                                    			// Variable for time that has passed
	float time_turning = 3.0;

    start = std::chrono::system_clock::now();                       // Start the timer
	ros::spinOnce();

    while (ros::ok() && seconds_elapsed <= time_turning && !AnyBumperPressed) { // Move forward some more
        vel_pub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        // if (AnyBumperPressed) {
        //     break;
        // }
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
	ros::Duration(2.0).sleep();
}


//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emotional_follower");
	ros::NodeHandle nh;
	
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	VelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
 	//ros::ServiceClient led_client = nh.serviceClient<led_manager::LedAnim>("led_anim");

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
	std::chrono::time_point<std::chrono::system_clock> reverse_start;
    std::chrono::time_point<std::chrono::system_clock> fear_start;
	std::chrono::time_point<std::chrono::system_clock> buffer_start;


	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	// geometry_msgs::Twist vel; //unnecessary, likely will be deleted
	// vel.angular.z = 0.2;
	// vel.linear.x = 0.0;
	// sc.playWave(path_to_sounds + "sound.wav");
	
	start = std::chrono::system_clock::now();
	// buffer_start = std::chrono::system_clock::now(); May cause issues
	// reverse_start = std::chrono::system_clock::now();
	WorldState = 0;
    uint64_t seconds_elapsed = 0;
	uint64_t time_in_reverse = 0;
	uint64_t time_in_buffer = 0;
	uint64_t time_alone = 0;
	uint8_t last_state = 0;
	bool is_alone = false;
	bool is_reversing = false;
	bool following_human = false;
	ros::spinOnce();
	ros::Duration(0.5).sleep();

	while(ros::ok() && seconds_elapsed <= 480){		
		ros::spinOnce();
		if (buffer_start) time_in_buffer = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-buffer_start).count();
		if (reverse_start) time_in_reverse = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-reverse_start).count();
		if (fear_start) time_alone = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-fear_start).count();
		
		if ((follow_cmd.linear.x != 0) || (follow_cmd.angular.z != 0)) following_human = true;

		if (!is_alone && !following_human){
			fear_start = std::chrono::system_clock::now();
			is_alone = true;
		}

		if (!is_reversing && FollowerInReverse){
			reverse_start = std::chrono::system_clock::now();
			is_reversing = true;
		}
		else if (MovingForward) reverse_start = std::chrono::system_clock::now();

		// Fear Mode Conditions
		if ((is_alone) && (time_in_buffer > BUFFER_TIME) && (time_alone > FEAR_TIME) && (last_state != 3)) WorldState = 1;

		// Sad Mode Conditions
		else if (AnyBumperPressed) WorldState = 2;

		// Excitement Mode Conditions
		else if ((following_human) && (last_state == 1)) WorldState = 3;

		// Rage Mode Conditions
		else if (FollowerInReverse && (time_in_reverse > REVERSE_TIME)) WorldState = 4;

		switch (WorldState) {
			case 0:
				neutralMode(VelPub); //idk play music or show images or something
				is_alone = true;
				break;
			case 1: 
				fearMode(VelPub);
				last_state = 1;
				break;
			case 2:
				sadMode(VelPub);
				buffer_start = std::chrono::system_clock::now();
				last_state = 2;
				is_alone = true;
				break;
			case 3:
				excitedMode(VelPub);
				buffer_start = std::chrono::system_clock::now();
				is_alone = true;
				last_state = 3;
				break;
			case 4:
				rageMode(VelPub);
				buffer_start = std::chrono::system_clock::now();
				is_alone = true;
				is_reversing = false;
				last_state = 4;
				break;
		}

		seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
