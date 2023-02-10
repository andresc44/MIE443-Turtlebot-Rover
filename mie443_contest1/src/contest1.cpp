// Advanced Main Algorithm
//MIE443 CONTEST 1  

// Function variables: variable_name
// Functions: functionName
// Definitions: DEFINITION_NAME
// Global variables: VariableName

// 2 empty lines between functions
// 1 empty line after if, else if, else, for, and while (unless followed by another '}' )
// Initialization of variables at top in order of types
// Comments aligned on function level
// Comments start with space and capital

//CONVENTION
//CCW rotation is positive
//forward is x direction
//left is y direction

//!!!!!!!LIBRARIES!!!!!!!!!!!!!!!!!!!!!#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>
#include <chrono>

//!!!!!!CONSTANTS!!!!!!!!!!!!!!!!!!!!!
#define N_BUMPER (3)                        // Number of bumpers on kobuki base
#define RAD2DEG(rad) ((rad) * 180. / M_PI)  // Coversion function 
#define DEG2RAD(deg) ((deg) * M_PI / 180.)  // Inverse conversion function
#define BACKWARD_V -0.05                    // Magnitude and direction in x axis
#define TURNING_V 0.5                       // Rad/s

//!!!!!!!!!!!GLOBAL VARIABLES!!!!!!!!!!!!!!!!!!!!!!!!!
uint8_t l_state[3] = {1,0,0};
uint8_t l_state[3] = {1,1,0};
uint8_t r_state[3] = {0,0,1};
uint8_t rm_state[3] = {0,1,1};
uint8_t m_state[3] = {0,1,0};
uint8_t lmr_state[3] = {1,1,1};
uint8_t lr_state[3] = {1,0,1};

uint8_t PressedBumper[3]={0,0,0};
uint8_t Bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
int32_t NLasers=0, DesiredNLasers=0, DesiredAngle=5;            // Laser parameters
float Angular = 0.0;                                            // Declare rotational vel (z) rad/s
float Linear = 0.1;                                             // Declare linear velocity (x) m/s
float PosX = 0.0, PosY = 0.0, Yaw = 0.0;                        // Initialize odom position
float MinLaserDist = std::numeric_limits<float>::infinity();    // Set minimum distance, volatile variable
float Dist = 0.5;
bool AnyBumperPressed = false; //reset variable to false

ros::Publisher VelPub;
// ros::Subscriber bumper_sub;
// ros::Subscriber laser_sub;
// ros::Subscriber odom;
geometry_msgs::Twist Vel; //create message for velocities as Twist type

// !!!!!!!!!!!!DECLARE FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!!
void reverse(ros::Publisher VelPub);
void rotate(ros::Publisher VelPub,int angle);

//!!!!!!!!!!CALLBACK FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    Bumper[msg->bumper] = msg->state;                           //assigns bumper array to match message based on msg message
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


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    MinLaserDist = std::numeric_limits<float>::infinity(); //no minimum Distance 
    NLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment; //count how many lasers there are  per cycle
    DesiredNLasers = DesiredAngle*M_PI / (180*msg->angle_increment); //offset desired
    // ROS_INFO("Size of laser scan array: %i and size of offset: %i", NLasers, DesiredNLasers); //print statement
    
    if (DesiredAngle * M_PI / 180 < msg->angle_max && -DesiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = NLasers / 2 - DesiredNLasers; laser_idx < NLasers / 2 + DesiredNLasers; ++laser_idx){
            MinLaserDist = std::min(MinLaserDist, msg->ranges[laser_idx]);
        }
    }
    
    else { //edge cases?
        for (uint32_t laser_idx = 0; laser_idx < NLasers; ++laser_idx) {
            MinLaserDist = std::min(MinLaserDist, msg->ranges[laser_idx]);
        }
    }

}


void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("odom");
    PosX = msg->pose.pose.position.x; //get x position
    PosY = msg->pose.pose.position.y; //get y position
    Yaw = tf::getYaw(msg->pose.pose.orientation); //covert from quaternion to Yaw
    tf::getYaw(msg->pose.pose.orientation); //not sure
    // ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", PosX, PosY, Yaw, RAD2DEG(Yaw)); //print statement
    
//fill with your code
}

//!!!!!!!!!USER-DEFINED FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!!!
bool array_equal(uint8_t array_1[], uint8_t array_2[]) {
    for (int i =0; i< 2; i++){
        if (array_1[i] != array_2[i] ) {
            return false;
        }
    }

    return true;
}


int forward(ros::Publisher VelPub, float linear_vel, float distance)
{
    ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;       //initialize timer

    Vel.angular.z = 0.0;
    Vel.linear.x = linear_vel;
    uint64_t secondsElapsed = 0;                                    //variable for time that has passed
    float time_forward = abs(distance/linear_vel);

    start = std::chrono::system_clock::now();                       //start the timer

    while (ros::ok() && secondsElapsed <= time_forward && !AnyBumperPressed) {
        VelPub.publish(Vel);
        ros::spinOnce();                                            //listen to all subscriptions once
        // if (AnyBumperPressed) {
        //     break;
        // }
        loop_rate.sleep();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
}


void reverse(ros::Publisher VelPub) {                               // Called if any bumper pressed
    ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;       // Initialize timer

    Vel.linear.x = BACKWARD_V;                                      // Set linear to Twist
    uint64_t seconds_elapsed = 0;                                   // New variable for time that has passed    
    int angle = 0;

    start = std::chrono::system_clock::now();                       // Start new timer at current time

    while (ros::ok() && seconds_elapsed <= 4) {                     // Kobuki diameter is 0.3515m, we want to travel half (3.5s x 0.05m/s) 
        VelPub.publish(Vel);                                        // Publish cmd_vel to teleop mux input
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
    }
    
    if (array_equal(PressedBumper,l_state)) angle = -45;            // Left bumper, rotate right (right is neg)
    
    else if (array_equal(PressedBumper, l_state)) angle = -90;     // Left and middle pressed only

    else if (array_equal(PressedBumper,r_state)) angle = 45;        // Right pressed only, rotate left (pos)

    else if (array_equal(PressedBumper,rm_state)) angle = 90;       // Right and middle pressed only, rotate left

    else if (array_equal(PressedBumper,m_state)) {                  // Middle pressed only
        // Rotate random angle
        int array_angles[2] = {90, -90};                            // Array of random angles
        int rand_num = rand() % 2;                                  // Random number from 0 to 1
        angle = array_angles[rand_num];                             // Index the array angles using random number, to rotate either left or right 90 degrees
    }

    else if  (array_equal(PressedBumper,lmr_state) || array_equal(PressedBumper,lr_state)) { // All pressed or left and right pressed
       // Rotate random angle
        int array_angles[2] = {135, -135};                          // Array of random angles
        int rand_num = rand() % 2;                                  // Random number from 0 to 1
        angle = array_angles[rand_num];                             // Index the array angles using random number, to rotate either left or right 135 degrees 
    }
   
    rotate(VelPub, angle);                                          // Rotate function
}

   
void rotate(ros::Publisher VelPub, int angle_rot){                  // Called with input as an angle in degree
    
   uint32_t angle_rad = DEG2RAD(angle_rot);                         // Function takes in an angle in degrees, converts it to rad
   uint32_t turning_time = angle_rad/TURNING_V;                     // Calculates the time needed to turn based on constant turning speed, and the angle
       
   std::chrono::time_point<std::chrono::system_clock> start;        // Initialize timer
   start = std::chrono::system_clock::now();                        // Start new timer at current time
   uint64_t seconds_elapsed = 0;                                    // New variable for time that has passed
    
   ros::Rate loop_rate(10);
   Vel.angular.z = TURNING_V;                                       // Set turning speed  
   
    while (ros::ok() && seconds_elapsed <= turning_time) {           // Turn until the turning time needed to complete the angle is passed   
        VelPub.publish(Vel);                                        // Start turning
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "image_listener"); //node name
    ros::NodeHandle nh;                      //initialize node handler

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback); //subscribe to bump topic with callback
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback); //subscribe  to /scan topic
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback); //NEW
    VelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); //publish cmd_velocity as Twist message

    ros::Rate loop_rate(10); //code will try to run at 10 Hz

    forward(VelPub, Linear, Dist);

    return 0;
}