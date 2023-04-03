#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

using namespace std;


#define TURNING_V 0.6;
#define BACKWARD_V -0.1  
                        



geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
	
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    Bumper[msg->bumper] = msg->state;                           // Assigns bumper array to match message based on msg message
    AnyBumperPressed = false;
    for (uint8_t i = 0; i < N_BUMPER; i++){
        if (Bumper[i] == kobuki_msgs::BumperEvent::PRESSED){
            // bumper[0] = leftState, bumper[1] = centerState, bumper[2] = rightState
            AnyBumperPressed = true;
            
        }

        
    }

    if (AnyBumperPressed) {
	    world_state = 2;
    }
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

			
			
		}else if(world_state == 2){ //sad
			
			
			///first reverse clear of the obstacle
			Vel.linear.x = BACKWARD_V;                                      // Set linear to Twist
			Vel.angular.z = 0;
    			uint64_t seconds_elapsed = 0;                                   // New variable for time that has passed    
   			Backward_time = 0.5/abs(BACKWARD_V);   //0.5 m backwards
    			start = std::chrono::system_clock::now();                       // Start new timer at current time

    			while (ros::ok() && seconds_elapsed <= Backward_time) {            // Kobuki diameter is 0.3515m, we want to travel half (3.5s x 0.05m/s) 
        			VelPub.publish(Vel);                                        // Publish cmd_vel to teleop mux input
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
			
				
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}




