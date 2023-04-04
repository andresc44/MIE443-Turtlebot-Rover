#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

using namespace std;
#define LOOP_RATE 10                            // Rate for while loops that dictate callback and publish frequency (Hz)
#define FORWARD_DIST 0.5

geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist Vel;                                       // Create message for velocities as Twist type
int world_state;
bool FollowerInReverse = false;
ros::Publisher VelPub;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
	FollowerInReverse = false;
	if (msg.linear.x < -0.1) FollowerInReverse = true;
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
}

// COPIED FROM CONTEST 1 //////////////////////////////////////////////////////////////////////////////////////
#include <kobuki_msgs/BumperEvent.h>

uint8_t Bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool AnyBumperPressed = false;                  // Reset variable to false
#define N_BUMPER (3)                            // Number of bumpers on kobuki base
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


void rotate(ros::Publisher VelPub, int angle_rot){                  // Called with input as an angle in degree
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
        VelPub.publish(Vel);                                        // Start turning
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rageMode(ros::Publisher VelPub, float linear_vel, float rot_vel) {
	// Shake a little
	// Publish forward in a loop until bumper is pressed
	// Make noises
	// Turn screen red 

	sc.playWave(path_to_sounds + "sound.wav"); //Get out of me swamp
	ros::Duration(0.5).sleep();

	ros::Rate loop_rate(LOOP_RATE);
    std::chrono::time_point<std::chrono::system_clock> start;       // Initialize timer

    Vel.angular.z = 0.0;
    Vel.linear.x = linear_vel;
    uint64_t secondsElapsed = 0;                                    // Variable for time that has passed


    while (ros::ok()  && !AnyBumperPressed) {
        VelPub.publish(Vel);
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
    Vel.linear.x = linear_vel;
    secondsElapsed = 0;                                    // Variable for time that has passed
	float time_forward = abs(FORWARD_DIST/linear_vel);

    start = std::chrono::system_clock::now();                       // Start the timer
	ros::spinOnce();

    while (ros::ok() && secondsElapsed <= time_forward && !AnyBumperPressed) { //Move forward
        VelPub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        // if (AnyBumperPressed) {
        //     break;
        // }
        loop_rate.sleep();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }

	Vel.angular.z = rot_vel;
    Vel.linear.x = 0.0;
    secondsElapsed = 0;                                    			// Variable for time that has passed
	float time_turning = 3.0;

    start = std::chrono::system_clock::now();                       // Start the timer
	ros::spinOnce();

    while (ros::ok() && secondsElapsed <= time_turning && !AnyBumperPressed) { // Move forward some more
        VelPub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        // if (AnyBumperPressed) {
        //     break;
        // }
        loop_rate.sleep();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
	ros::Duration(2.0).sleep();
}


//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emotional_follower");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	VelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
	std::chrono::time_point<std::chrono::system_clock> reverse_start;
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

	uint64_t time_in_reverse = 0;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		if (FollowerInReverse){
			reverse_start = std::chrono::system_clock::now();
			reverse_started = 
		}
		time_in_reverse = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-reverse_start).count();

		if (time_in_reverse > 10) world_state = 4;

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
		else if (world_state == 4) {
			rageMode(VelPub, 0.5, 0.3);
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
