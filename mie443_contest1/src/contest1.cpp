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
#include "std_msgs/String.h"


#include <stdio.h>
#include <cmath>
#include <chrono>
#include <vector>
#include <math.h>

//!!!!!!CONSTANTS!!!!!!!!!!!!!!!!!!!!!
#define KOBUKI_DIAM 0.3515                      // Diameter of Turtlebot 
#define N_BUMPER (3)                            // Number of bumpers on kobuki base
#define RAD2DEG(rad) ((rad) * 180. / M_PI)      // Coversion function 
#define DEG2RAD(deg) ((deg) * M_PI / 180.)      // Inverse conversion function
#define SCAN_LENGTH 640                         // Elements in an array of scan, last index is SCAN_LENGTH-1
#define B_OFFSET 198                            // Indeces difference from A to either B
#define A_THRESHOLD 0.617                       // Considered obstacle directly in front
#define B_THRESHOLD 0.65                        // Considered wall in the window for passage
#define C_THRESHOLD 0.7788                      // Side (30 degree) distances to check for free space
#define ACF_THRESHOLD 1      //!!!!!!!          // Distance directly ahead to start slowing down by
#define ODOM_ARRAY_LENGTH 50                    // Max number of checkpoints per trial
#define FORWARD_FAST_V 0.6   //!!!!!            // Velocity when far from walls
#define FORWARD_SLOW_V 0.3   //!!!!!            // Velocity near walls
#define STEP_SIZE 0.15                          // Size of Forward steps (m)
#define BACKWARD_V -0.2                         // Magnitude and direction in x axis
#define BACKWARD_T 0.88                         // Move backwards a distance == to the radius of the turtlebot
#define LOOP_RATE 10                            // Rate for while loops that dictate callback and publish frequency (Hz)
#define TURNING_V 1         //!!!!!             // Rad/s
#define MAX_OBST_DIST 0.5                       // If less than this, we should turn, meters
#define STOPS_SPACING 2.5     //!!!!!           // Distance between checkpoints, meters
#define TURNING_LIM 500                         // Amounts of total degrees turned before turning robot around
#define OCCUP_WEIGHT 10                         // Weights for each factor
#define ODOM_WEIGHT 1
#define SCAN_WEIGHT 10
#define ADJUST_ANGLE 15                         // How much to adjust robot by when wall detected on sides, degrees
#define CONSECUTIVE_TILTS 8                     // Number of consecutive tilts to occur before thinking of it as a mistake
// #define CHECKPOINT_RADIUS 0.2                // How close are we to a previous checkpoint, may remove
const int CENTRE_INDEX = SCAN_LENGTH / 2 -1;    // Get index for the middle index for the A value


//!!!!!!!!!!!GLOBAL VARIABLES!!!!!!!!!!!!!!!!!!!!!!!!!
uint8_t LState[3] = {1,0,0};
uint8_t LMState[3] = {1,1,0};
uint8_t RState[3] = {0,0,1};
uint8_t RMState[3] = {0,1,1};
uint8_t MState[3] = {0,1,0};
uint8_t LMRState[3] = {1,1,1};
uint8_t LRState[3] = {1,0,1};

uint8_t PressedBumper[3]={0,0,0};               // 3 bit array of bumper status
std::vector<float> CurrOdom{0, 0};              // Where the turtlebot is in that moment
// std::vector<float> XboxRanges = {0};         // Storage for scan topic messages
std::vector<std::vector<float>> OdomArray{ODOM_ARRAY_LENGTH, std::vector<float>(2)}; // Full Array saving all checkpoint locations

bool IsNearWall = true;                         // Can we go fast or not
bool AnyBumperPressed = false;                  // Reset variable to false
int TurningBias = 0;                            // Finds clearest path or correction direction

uint8_t Bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t LeftState = Bumper[kobuki_msgs::BumperEvent::LEFT];     // kobuki_msgs::BumperEvent::PRESSED if bumper is pressed, kobuki_msgs::BumperEvent::RELEASED otherwise
int32_t NLasers=0, DesiredNLasers=0, DesiredAngle=5;            // Laser parameters
float Angular = 0.0;                                            // Declare rotational vel (z) rad/s
float Linear = 0.0;                                             // Declare linear velocity (x) m/s
float PosX = 0.0, PosY = 0.0, Yaw = 0.0;                        // Initialize odom position
float MinLaserDist = std::numeric_limits<float>::infinity();    // Set minimum distance, volatile variable
float CMinus = 0;
float BMinus = 0;
float ACentre = 0;
float BPlus = 0;
float CPlus = 0;

ros::Publisher VelPub;
ros::Publisher OutputPub;
geometry_msgs::Twist Vel;                                       // Create message for velocities as Twist type
std_msgs::String Message;
std::stringstream StringContent;

// !!!!!!!!!!!!DECLARE FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!!
void reverse(ros::Publisher VelPub);
void rotate(ros::Publisher VelPub, int angle);

//!!!!!!!!!!CALLBACK FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!
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


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS_INFO("ranges: %f", msg->ranges[0]);
    CMinus = msg->ranges[0];
    BMinus = msg->ranges[CENTRE_INDEX - B_OFFSET];
    ACentre = msg->ranges[CENTRE_INDEX];
    BPlus = msg->ranges[CENTRE_INDEX + B_OFFSET];
    CPlus = msg->ranges[SCAN_LENGTH-1];
}


void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    CurrOdom[0] = msg->pose.pose.position.x;                // Get x position
    CurrOdom[1] = msg->pose.pose.position.y;                // Get y position
    Yaw = RAD2DEG(tf::getYaw(msg->pose.pose.orientation));  // Covert from quaternion to Yaw
    // ROS_INFO("IN ODOM LOOP, x pose: %f",  msg->pose.pose.position.x);
    // ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", PosX, PosY, Yaw, RAD2DEG(Yaw));
}


//!!!!!!!!!USER-DEFINED FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!!!
bool arrayEqual(uint8_t array_1[], uint8_t array_2[]) {
    for (int i =0; i< 2; i++){
        if (array_1[i] != array_2[i] ) {
            return false;
        }
    }

    return true;
}

int sign(float number) {
    return (number>0)? 1: -1;                               // Positive -> 1, otherwise -1
}


int dotAndSign(float vector1[2], float vector2[2]) {
    float product = vector1[0]*vector2[0] + vector1[1]*vector2[1];
    return sign(product);                                   // Dot product and its sign
}


float distance(std::vector<float> prev_odom, std::vector<float> new_odom){     // Magnitude of change in distance between odom coordinates
    // ROS_INFO("new_x: %f, old_x: %f", new_odom[0], prev_odom[0]);
    float A = (new_odom[0] - prev_odom[0]);
    float B = (new_odom[1] - prev_odom[1]);
    float C_squared = A*A + B*B;
    // ROS_INFO("A: %f, B: %f, C_squared: %f", A, B, C_squared);
    return (std::sqrt(C_squared));    
}


int forward(ros::Publisher VelPub, float linear_vel, float distance)
{
    // if (linear_vel == float(FORWARD_FAST_V)) ROS_INFO("forward fast!");
    // else if (linear_vel == float(FORWARD_SLOW_V)) ROS_INFO("forward slow");
    // else ROS_INFO("Forward but unsure why, linear_vel is: %f", linear_vel);
    ros::Rate loop_rate(LOOP_RATE);
    std::chrono::time_point<std::chrono::system_clock> start;       // Initialize timer

    Vel.angular.z = 0.0;
    Vel.linear.x = linear_vel;
    uint64_t secondsElapsed = 0;                                    // Variable for time that has passed
    float time_forward = abs(distance/linear_vel);

    start = std::chrono::system_clock::now();                       // Start the timer

    while (ros::ok() && secondsElapsed <= time_forward && !AnyBumperPressed) {
        VelPub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        // if (AnyBumperPressed) {
        //     break;
        // }
        loop_rate.sleep();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
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
    
   ros::Rate loop_rate(LOOP_RATE);
   Vel.linear.x = 0;
   Vel.angular.z = dir * TURNING_V;                                 // Set turning speed  
//    ROS_INFO("direction: %f", dir*TURNING_V);
    while (ros::ok() && seconds_elapsed <= turning_time) {          // Turn until the turning time needed to complete the angle is passed   
        VelPub.publish(Vel);                                        // Start turning
        loop_rate.sleep();
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
}


uint8_t determineScanType() { // Detect environment classification based on scan data
    uint8_t state = 0;
    IsNearWall = true;
    TurningBias = 0;
    bool CM = 0;
    bool BM = 0;
    bool AC = 0;
    bool BP = 0;
    bool CP = 0;

    bool AC_far = 0;

    if (CMinus > C_THRESHOLD) CM = 1;
    if (BMinus > B_THRESHOLD) BM = 1;
    if (ACentre > A_THRESHOLD) AC = 1;
    if (BPlus > A_THRESHOLD) BP = 1;
    if (CPlus > C_THRESHOLD) CP = 1;

    if (ACentre > ACF_THRESHOLD) AC_far = 1;

    // ROS_INFO("5 points left to right: %i, %i, %i, %i, %i", CP, BP, AC, BM, CM);


    if (CM && BM && AC_far && BP) {
        state = 1;                      // Forward fast
        IsNearWall = false;
    }

    else if (BM && AC && BP) {
        state = 2;                      // Forward slow
        if (!AC_far) ROS_INFO("far distance is blocked, going slower");
        else if (!CM && !CP) ROS_INFO("C minus and plus are not clear");
        else if (!CM) ROS_INFO("Just C minus blocked");
        else if (!CP) ROS_INFO("Just C plus blocked");
    }

    else if (AC && BP) {
        state = 3;
        TurningBias = 1;                // Tilt left
    }

    else if (AC && BM) {
        state = 4;
        TurningBias = -1;               // Tilt right
    }

    else if ((!AC && !BP) || (!AC && !BM) || (!BM && !BP)) {
        state = 5;
        if (CP) TurningBias += 1;       // Turn left
        ROS_INFO("Open space left");
        if (CM) TurningBias = -1;       // Turn right
        ROS_INFO("Open space right");
        
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

    for (int i = 1; i++; i < ODOM_ARRAY_LENGTH) {
        x_i = OdomArray[i][0];
        y_i = OdomArray[i][1];
        if (x_i == 0 && y_i == 0){
            break;
        }

        x_sum += x_i;
        y_sum += y_i;
        len += 1;
    }

    float x_centroid = x_sum / len;
    float y_centroid = y_sum / len;

    float difference[2] = {(CurrOdom[0] - x_centroid), (CurrOdom[1] - y_centroid)};
    
    float yaw_rad_left = DEG2RAD(Yaw + 90);
    float unit_vector_left[2] = {cosf(yaw_rad_left), sinf(yaw_rad_left)};
    int is_left_away_from_centroid = dotAndSign(unit_vector_left, difference);
    return is_left_away_from_centroid;
}

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 2 Functions below are untested and contain bugs

// #define SIDE_DISTANCE 0.4           // Distance to check to our lefts for unexplored areas (m)
// #define BLOCK_SIZE 2                // Area checked for occupancy is of size (BLOCK_SIZE * 2 + 1) ^2
// #define TOTAL_BLOCKS (BLOCK_SIZE * 2 + 1) * (BLOCK_SIZE * 2 + 1)
// #define UNEXPLORED_THRESH 0.7       // Max Threshold for whether a given area has been explored
// #include <nav_msgs/OccupancyGrid.h>

// uint64_t GridX = 0;
// uint64_t GridY = 0;
// float MapResolution = 0;
// long int MapWidth = 1;
// long int MapHeight = 1;

// //global pointer set to null
// std::vector<std::vector<float>> *MapPointer = new std::vector<std::vector<float>>;


// void occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {
//     // ROS_INFO("ranges: %f", msg->ranges[0]);
//     // int occup_array[] = msg->data;
//     tf::TransformListener listener;
//     tf::StampedTransform transform;

//     MapResolution =msg->info.resolution;
//     MapWidth = msg->info.width;
//     MapHeight = msg->info.height;
//     map_array = msg->data;

//     std::vector<std::vector<float>> map_array{MapWidth, std::vector<float>(MapHeight)}; //!!!This has to be fixed
//     map_array = msg->data;

//     //assign global pointer to point at vector above
//     MapPointer = &map_array; // Not exactly this

//     float origin_x = msg->info.origin.position.x;
//     float origin_y = msg->info.origin.position.y;
//     // float origin_yaw = RAD2DEG(tf::getYaw(msg->info.origin.quaternion));

//     try listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

//     catch (tf::TransformException &ex) {
//         ROS_ERROR("%s",ex.what());
//         ros::Duration(1.0).sleep();
//         continue;
//     }

//     float map_x = transform.getOrigin().x();
//     float map_y = transform.getOrigin().y();

//     // must get map_x and map_y from transform
//     GridX = (unsigned int)((map_x - origin_x) / MapResolution)
//     GridY = (unsigned int)((map_y - origin_y) / MapResolution)
    
// }


// int newFrontier() {
//     long int surrounding_x = 0;
//     long int surrounding_y = 0;
//     float left_counter = 0;
//     float right_counter = 0;
//     bool left_in_bounds = true;
//     bool right_in_bounds = true;
//     int occup_dec = 0;

//     long int matrix_x = GridX //Need to change
//     long int matrix_y = Gridy //Same

//     float x_delta = SIDE_DISTANCE * sinf(Yaw);
//     float y_delta = SIDE_DISTANCE * cosf(Yaw);
//     int x_disc = (int)(x_delta/MapResolution);
//     int y_disc = (int)(y_delta/MapResolution);

//     long int left_index_x = matrix_x + x_disc;
//     long int left_index_y = matrix_y - y_disc;
//     long int right_index_x = matrix_x - x_disc;
//     long int right_index_y = matrix_y + y_disc;

//     // bool surrounding_in_bounds = true;

//     if ((left_index_x < 0) || (left_index_y < 0) || (left_index_x > MapWidth) || (left_index_y > MapHeight)) {
//         //Left is out of map bounds
//         left_in_bounds = false;        
//     }

//     else long int left_centre[2] = {left_index_x, left_index_y};

//     if ((right_index_x < 0) || (right_index_y < 0) || (right_index_x > MapWidth) || (right_index_y > MapHeight)) {
//         //Right is out of map bounds
//         right_in_bounds = false;        
//     }

//     else long int right_centre[2] = {right_index_x, right_index_y};

//     for (int i = -BLOCK_SIZE; i <= BLOCK_SIZE; i++) {
//         for (int j = -BLOCK_SIZE; j <= BLOCK_SIZE; j++) {
//             // surrounding_in_bounds = true;
//             if (left_in_bounds) {          
//                 surrounding_x = left_centre[0] + i;
//                 surrounding_y = left_centre[1] + j;

//                 if ((surrounding_x >= 0) && (surrounding_y >= 0) && (surrounding_x <= MapWidth) && (surrounding_y <= MapHeight)) {
//                     // If the dot in the left surounding area is a valid point in map
//                     left_counter += map_array[surrounding_x, surrounding_y]
//                 }
//             }

//             if (right_in_bounds) {          
//                 surrounding_x = right_centre[0] + i;
//                 surrounding_y = right_centre[1] + j;

//                 if ((surrounding_x >= 0) && (surrounding_y >= 0) && (surrounding_x <= MapWidth) && (surrounding_y <= MapHeight)) {
//                     // If the dot in the right surounding area is a valid point in map
//                     right_counter += map_array[surrounding_x, surrounding_y]
//                 }
//             }
//         }
//     }

//     float left_avg = left_counter / TOTAL_BLOCKS;
//     float right_avg = right_counter / TOTAL_BLOCKS;

//     if (left_avg < UNEXPLORED_THRESH) occup_dec += 1;
//     if (right_avg < UNEXPLORED_THRESH) occup_dec -= 1;

//     return occup_dec;
// }

int newFrontier() {
    return 0;
}


// !!!!!!!!!!!!!MAIN!!!!!!!!!!!!!!!!!!!!!!!!!!
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");                    // Node name
    ros::NodeHandle nh;                                         // Iinitialize node handler
    ros::Rate loop_rate(LOOP_RATE);                             // Code will try to run at LOOP_RATE Hz

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback); //subscribe to bump topic with callback
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);       // Subscribe  to /scan topic
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odomCallback);
    // ros::Subscriber map_sub = nh.subscribe("map", 1, &occupancyCallback);
    VelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); // Publish cmd_velocity as Twist message
    OutputPub = nh.advertise<std_msgs::String>("feedback", 1000);
    uint64_t seconds_elapsed = 0;                               // Variable for time that has passed
    uint8_t scan_mode = 0;                                      // List of X states
    uint8_t bad_tilt_counter = 0;                               // Number of consecutive tilts done
    int checkpoint_counter = 1;             
    float rotation = 0;                             
    float dist_travelled = 0;                                   // How much moved since last checkpoint
    long turning_count = 0;                                     // Accumulation of how much travelled
    bool just_rotated = false;                                  // Did we just make a 90 degree turn?
    bool is_in_corner = false;                                  // Did we just make a 90 degree turn?
    int decision = 0;
    int scan_cmd = 0;
    int occup_cmd = 0;
    int odom_cmd = 0;
    long int seconds_long = 0;
    long int loop_iterations = 0;



    ros::spinOnce();
    std::vector<float> last_odom = CurrOdom;                          // Location of last point
    OdomArray[0] = last_odom;
    
    geometry_msgs::Twist vel;                                   // Create message for velocities as Twist type
    std::chrono::time_point<std::chrono::system_clock> start;   // Initialize timer
    start = std::chrono::system_clock::now();                   // Start the timer

    while(ros::ok() && seconds_elapsed <= 480) {        // While ros master running and < 8 minutes
        ros::spinOnce();                                // Listen to all subscriptions once
        loop_iterations += 1;
        if (loop_iterations < 5) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;

        }
        seconds_long = seconds_elapsed;
        ROS_INFO("\nTime: %ld, Pos: (%f, %f),  Ori: %f deg, turning_cnt: %ld", seconds_long, CurrOdom[0], CurrOdom[1], Yaw, turning_count); //print current state of everything, !!!!!!!CHANGED 2 LINES
        dist_travelled = distance(last_odom, CurrOdom); // Distance is function to determine distance between 2 coordinates
        //TO-DO!!!!!!!!!!
        //add criteria to do checkpoint of forward happens uninterreupted
        //TO-DO!!!!!!!!!!

        if (dist_travelled > STOPS_SPACING) {           // Checks space since last checkpoint
            ROS_INFO("New checkpoint location, doing 360");
            last_odom = CurrOdom;                       // Reset reference location
            dist_travelled = 0;
            OdomArray[checkpoint_counter] = CurrOdom;   // Keeps track of locations of all past checkpoints, leaves first index as 0,0
            checkpoint_counter += 1;
            rotate(VelPub, 360);                        // Scan area. positive is counterclockwise
            // rotate(VelPub, 600); // Just for simulation
            continue;
        }
        
        scan_mode = determineScanType();                // Recognize different possibilities based on scan array, also look at changes in odom
        ROS_INFO("Scan Mode: %i", scan_mode);

        // !!!!!Print message to topic: feedback
        StringContent << "Scan Mode: " << scan_mode;
        Message.data = StringContent.str();
        OutputPub.publish(Message);

        if (scan_mode < 3) {
            if (IsNearWall) {
                ROS_INFO("Forward slow");
                forward(VelPub, FORWARD_SLOW_V, STEP_SIZE);
            }

            else {
                ROS_INFO("Forward Fast");
                forward(VelPub, FORWARD_FAST_V, STEP_SIZE);

            }
            // IsNearWall? forward(VelPub, FORWARD_SLOW_V, STEP_SIZE): forward(VelPub, FORWARD_FAST_V, STEP_SIZE);
            

            bad_tilt_counter = 0;
            just_rotated = false;
            is_in_corner = false;
            loop_rate.sleep();
        }

        else if ((scan_mode == 3) || (scan_mode == 4)){ //Tilt
            ROS_INFO("Minor tilt");
            just_rotated = false;
            is_in_corner = false;
            if (bad_tilt_counter >= CONSECUTIVE_TILTS) {
                reverse(VelPub);
                rotate(VelPub, 180);
                ROS_INFO("I'm stuck in a tilt loop, turning around now");
                turning_count = 0;
                continue;
            }

            rotation = TurningBias * ADJUST_ANGLE;
            rotate(VelPub, rotation);                   // Fix allignment by ~ 15 degrees depending on direction of bias
            turning_count += rotation;                  // Keep track of turning
            bad_tilt_counter += 1;                      // Track how many times we've done a consecutive tilt
        }

        else if (scan_mode == 5) {                      // Front wall
            bad_tilt_counter = 0;
            if (just_rotated) {                         // Bad turning decision, we're at a corner and turned into other wall
                ROS_INFO("I think I'm in a corner and going the other way");
                rotate(VelPub, 180);                    // Turn around, think about a dead end corridor situation
                is_in_corner = true;
                turning_count += -decision*180;         // If we made the wrong choice before, we want to overwrite that contribution to the count and add accordingly
                just_rotated = false;                   // Reset flag
                
                continue;
            }

            else if (is_in_corner) {
                ROS_INFO("I think I just hit a 3 way dead end and am going back");
                rotate(VelPub, -decision*90);
                turning_count += decision*90; 
                is_in_corner = false;
                continue;
            }

            if (turning_count > TURNING_LIM) {          // We've been turning too much
                ROS_INFO("I think I rotated too much left, turning around");
                rotate(VelPub, 180);                    // Go other direction
                turning_count = 0;                      // Reset flag
                continue;
            }

            else if (turning_count < -TURNING_LIM) {    // Same as above in other direction
                ROS_INFO("I think I rotated too much right, turning around");
                rotate(VelPub, 180);
                turning_count = 0;
                continue;
            }

            scan_cmd = TurningBias;                     // Based on clearest path, L or R, function
            odom_cmd = nearestCheckpoint();             // Based on nearby odom checkpoints, function, checks OdomArray
            occup_cmd = newFrontier();                  // Based on nearby occupancy grid, function
            // odom_cmd = 0;
            // occup_cmd = 0;



            decision = sign(scan_cmd*SCAN_WEIGHT + odom_cmd*ODOM_WEIGHT + occup_cmd*OCCUP_WEIGHT); // Weighted decision of costs
            ROS_INFO("Scan: %i, Odom: %i,  Occupancy: %i, Decision: %i", scan_cmd, odom_cmd, occup_cmd, decision);
            //TO-DO!!!!!!!!!!
            //rather than weight, potentially do steps to see if that area's been explored
            //TO-DO!!!!!!!!!!
            rotation = 90*decision;
            rotate(VelPub, rotation);                   // Rotate 90 based on decision
            turning_count += rotation;                  // Add rotation to count
            just_rotated = true;
        }

        else {
            rotate(VelPub, 180); //robot is confused
            turning_count = 0;
            bad_tilt_counter = 0;
        }

        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
        loop_rate.sleep();                              // Delay function for 100ms
    }
    // delete MapPointer;
    return 0;
}
