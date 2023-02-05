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

//!!!!!!!LIBRARIES!!!!!!!!!!!!!!!!!!!!!
#include <ros/console.h>
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
#define SCAN_LENGTH 32                      // Elements in an array of scan
#define ODOM_ARRAY_LENGTH 50                // Max number of checkpoints per trial
#define FORWARD_FAST_V 0.15                 // Velocity when far from walls
#define FORWARD_SLOW_V 0.08                 // Velocity near walls
#define BACKWARD_V -0.05                    // Magnitude and direction in x axis
#define MAX_OBST_DIST 0.5                   // If less than this, we should turn, meters
#define STOPS_SPACING 0.5                   // Distance between checkpoints, meters
#define TURNING_LIM 3000                    // Amounts of total degrees turned before turning robot around
#define OCCUP_WEIGHT 10                     // Weights for each factor
#define ODOM_WEIGHT 10
#define SCAN_WEIGHT 10
#define ADJUST_ANGLE 15                     // How much to adjust robot by when wall detected on sides, degrees
#define CHECKPOINT_RADIUS 0.2;              // How close are we to a previous checkpoint, may remove


//!!!!!!!!!!!GLOBAL VARIABLES!!!!!!!!!!!!!!!!!!!!!!!!!
uint8_t PressedBumper[3]={0,0,0};           // 3 bit array of bumper status
float CurrOdom[2] = {0, 0};                 // Where the turtlebot is in that moment
float XboxRanges[SCAN_LENGTH] = {};         // Storage for scan topic messages
float OdomArray[ODOM_ARRAY_LENGTH][2] = {}; // Full Array saving all checkpoint locations
bool IsNearWall = true;                     // Can we go fast or not
bool AnyBumperPressed = false;              // Reset variable to false
int TurningBias = 0;                        // Finds clearest path or correction direction

uint8_t Bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t LeftState = Bumper[kobuki_msgs::BumperEvent::LEFT];     // kobuki_msgs::BumperEvent::PRESSED if bumper is pressed, kobuki_msgs::BumperEvent::RELEASED otherwise
int32_t NLasers=0, DesiredNLasers=0, DesiredAngle=5;            // Laser parameters
float Angular = 0.0;                                            // Declare rotational vel (z) rad/s
float Linear = 0.0;                                             // Declare linear velocity (x) m/s
float PosX = 0.0, PosY = 0.0, Yaw = 0.0;                        // Initialize odom position
float MinLaserDist = std::numeric_limits<float>::infinity();    // Set minimum distance, volatile variable


//!!!!!!!!!!CALLBACK FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{   
    //Put in Selin's callback function
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //!!!!!TO-DO!!!!!!!!
    //Fix this function to output more meaningful data
    //!!!!!TO-DO!!!!!!!!
    MinLaserDist = std::numeric_limits<float>::infinity();                                      // No minimum distance 
    NLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;                         // Count how many lasers there are  per cycle
    DesiredNLasers = DesiredAngle*M_PI / (180*msg->angle_increment);                            // Offset desired
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", NLasers, DesiredNLasers);   // Print statement
    
    XboxRanges = msg->ranges;                                                                   // Make the scan results accessible globally

    if (DesiredAngle * M_PI / 180 < msg->angle_max && -DesiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = NLasers / 2 - DesiredNLasers; laser_idx < NLasers / 2 + DesiredNLasers; ++laser_idx){
            MinLaserDist = std::min(MinLaserDist, msg->ranges[laser_idx]);
        }
    }

    else {                                                                                      // Edge cases?
        for (uint32_t laser_idx = 0; laser_idx < NLasers; ++laser_idx) {
            MinLaserDist = std::min(MinLaserDist, msg->ranges[laser_idx]);
        }
    }
}


void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    CurrOdom[0] = msg->pose.pose.position.x;                // Get x position
    CurrOdom[1] = msg->pose.pose.position.y;                // Get y position
    Yaw = RAD2DEG(tf::getYaw(msg->pose.pose.orientation));  // Covert from quaternion to Yaw
    // ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", PosX, PosY, Yaw, RAD2DEG(Yaw));
}


//!!!!!!!!!USER-DEFINED FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!!!
int sign(float number) {
    return (number>0)? 1: -1;                               // Positive -> 1, otherwise -1
}


int dotAndSign(float vector1[2], float vector2[2]) {
    float product = vector1[0]*vector2[0] + vector1[1]*vector2[1];
    return sign(product);                                   // Dot product and its sign
}


float distance (float prev_odom[2], float new_odom[2]){     // Magnitude of change in distance between odom coordinates
    return (((new_odom[0] - prev_odom[0])**2 +(new_odom[1] - prev_odom[1])**2)**0.5);
}


uint8_t determineScanType (float xbox_distances[SCAN_LENGTH]) { // Detect environment classification based on scan data
    uint8_t state = 0;
    IsNearWall = true;
    TurningBias = 0;
    if (nothing anwywhere) {
        state = 1;                      // Forward fast
        IsNearWall = false;
    }

    else if (nothing in front, but walls around) {
        state = 2;                      // Forward slow
    }

    else if (tilt left) {
        state = 3;
        TurningBias = 1;                // Tilt left
    }

    else if (tilt right) {
        state = 4;
        TurningBias = -1;               // Tilt right
    }

    else if (wall in front) {
        state = 5;
        if (clearing leftwards){
            TurningBias = 1;            // Turn left
        }

        else if (clearing rightwards){
            TurningBias = -1;           // Turn right
        }
    }

    else {
        state = 6;                      // Confused
    }

    return state;
}


int nearestCheckpoint() {                           // Which direction will guide turtlebot away from centroid of checkpoints
    float x_sum = 0;
    float y_sum = 0;
    float x_i = 0;
    float y_i = 0;
    uint8_t len = 1;

    for (int i = 1, i++, i < ODOM_ARRAY_LENGTH) {
        x_i = OdomArray[i, 0];
        y_i = OdomArray[i, 1];
        if (x_i == 0 && y_i == 0){
            break
        }

        x_sum += x_i;
        y_sum += y_i;
        len += 1;
    }

    float x_centroid = x_sum / len;
    float y_centroid = y_sum / len;
    float difference[2] = [(CurrOdom[0] - x_centroid), CurrOdom[1] - y_centroid];
    
    float yaw_rad_left = DEG2RAD(Yaw + 90);
    float unit_vector_left = [cos(yaw_rad_left), sin(yaw_rad_left)];
    int is_left_away_from_centroid = dotAndSign(unit_vector, difference);
    return is_left_away_from_centroid;
}


int newFrontier() {
    //!!!!!!TO-DO!!!!!!
    // Check Occupancy Grid map
    //!!!!!!TO-DO!!!!!!
}


// !!!!!!!!!!!!!MAIN!!!!!!!!!!!!!!!!!!!!!!!!!!
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Navigator");         // Node name
    ros::NodeHandle nh;                         // Initialize node handler
    ros::Rate loop_rate(10);                    // Code will try to run at 10 Hz

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback); 
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);                          
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); //Publisher of velocities as Twist messages to teleop topic

    uint64_t seconds_elapsed = 0;               // Variable for time that has passed
    uint8_t scan_mode = 0;                      // List of X states
    uint8_t checkpoint_counter = 1;             
    float rotation = 0;                             
    float dist_travelled = 0;                   // How much moved since last checkpoint
    float last_odom[2] = {0,0};                 // Location of last point
    long turning_count = 0;                     // Accumulation of how much travelled
    bool just_rotated = false;                  // Did we just make a 90 degree turn?
    int decision = 0;
    int scan_cmd = 0;
    int occup_cmd = 0;
    int odom_cmd = 0;

    geometry_msgs::Twist vel;                                   // Create message for velocities as Twist type
    std::chrono::time_point<std::chrono::system_clock> start;   // Initialize timer
    start = std::chrono::system_clock::now();                   // Start the timer

    while(ros::ok() && seconds_elapsed <= 480) {        // While ros master running and < 8 minutes
        ros::spinOnce();                                // Listen to all subscriptions once
        ROS_INFO("Time: %i, Postion: (%f, %f),  Orientation: %f, degrees turning_count: %i, Range: %f", seconds_elapsed, CurrOdom[0], CurrOdom[1], Yaw, turning_count, MinLaserDist); //print current state of everything, !!!!!!!CHANGED 2 LINES
        dist_travelled = distance(last_odom, CurrOdom); // Distance is function to determine distance between 2 coordinates
        //TO-DO!!!!!!!!!!
        //add criteria to do checkpoint of forward happens uninterreupted
        //TO-DO!!!!!!!!!!
        if (dist_travelled > STOPS_SPACING) {           // Checks space since last checkpoint
            last_odom = CurrOdom;                       // Reset reference location
            dist_travelled = 0;
            OdomArray[checkpoint_counter] = CurrOdom;   // Keeps track of locations of all past checkpoints, leaves first index as 0,0
            checkpoint_counter += 1;
            rotate(360);                                // Scan area. positive is counterclockwise
            continue
        }

        scan_mode = determineScanType();                //recognize different possibilities based on scan array, also look at changes in odom
        ROS_INFO("Scan Mode: %i", scan_mode);
        if (scan_mode < 3) {
            IsNearWall? forward(FORWARD_SLOW_V): forward(FORWARD_FAST_V);  
        }

        else if ((scan_mode == 3) || (scan_mode == 4)){ //Tilt
            rotation = TurningBias*ADJUST_ANGLE;
            rotate(rotation)                            // Fix allignment by ~ 15 degrees depending on direction of bias
            turning_count += rotation;                  // Keep track of turning
        }

        else if (scan_mode == 5) {                      // Front wall
            if (just_rotated) {                         // Bad turning decision, we're at a corner and turned into other wall
                rotate(180);                            // Turn around, think about a dead end corridor situation
                is_in_corner = true;
                turning_count += -decision*180;         // If we made the wrong choice before, we want to overwrite that contribution to the count and add accordingly
                just_rotated = false;                   // Reset flag
                continue
            }

            else if (is_in_corner) {
                rotate(-decision*90);
                turning_count += decision*90; 
                is_in_corner = false;
                continue
            }

            if (turning_count > TURNING_LIM) {          // We've been turning too much
                rotate(180);                            // Go other direction
                turning_count = 0;                      // Reset flag
                continue
            }

            else if (turning_count < -TURNING_LIM) {    // Same as above in other direction
                rotate(180);
                turning_count = 0;
                continue
            }

            scan_cmd = TurningBias;                     // Based on clearest path, L or R, function
            odom_cmd = nearestCheckpoint();             // Based on nearby odom checkpoints, function, checks OdomArray
            occup_cmd = newFrontier();                  // Based on nearby occupancy grid, function

            decision = sign(scan_cmd*SCAN_WEIGHT + odom_cmd*ODOM_WEIGHT + occup_cmd*OCCUP_WEIGHT); //weighted decision of costs
            ROS_INFO("Scan: %f, Odom: %f,  Occupancy: %f, Decision: %f", scan_cmd, odom_cmd, occup_cmd, decision);
            //TO-DO!!!!!!!!!!
            //rather than weight, potentially do steps to see if that area's been explored
            //TO-DO!!!!!!!!!!
            rotation = 90*decision;
            rotate(rotation);                           // Rotate 90 based on decision
            turning_count += rotation;                  // Add rotation to count
            just_rotated = true;
        }

        else {
            rotate(180); //robot is confused
            turning_count = 0;
        }

        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
        loop_rate.sleep();                              // Delay function for 100ms
    }
    return 0;
}
