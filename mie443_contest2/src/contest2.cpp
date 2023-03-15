// MIE443 CONTEST 2
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
#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>

#include <stdio.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <math.h>
#include <tf2/Quaternion.h>
#include <nav_msgs/GetPlan.h>

//!!!!!!CONSTANTS!!!!!!!!!!!!!!!!!!!!!
#define LOOP_RATE 10                                // Rate for while loops that dictate callback and publish frequency (Hz)
#define VISION_RADIUS 0.3                           // The distance from the centre of the turtlebot to the centre of the cereal box when parked and scanning
#define ANGLE_INCREMENT 15                          // Degrees by which to increase in search for optimal goal destination from the centre
#define MAX_ANGLE 75                                // Max possible angle to deviate from centre optimal spot
#define CAMERA_LIMIT 20                             // Seconds to be stuck trying to read an image
#define ARRIVAL_DELAY 1                             // Time to wait before starting to try to scan camera
#define GOAL_TOLERANCE 0.03                         // Tolerance for goal destination when trying to make plan (x, and y)

// !!!!!!!!!!!!!MAIN!!!!!!!!!!!!!!!!!!!!!!!!!!
int main(int argc, char** argv) {
    ros::init(argc, argv, "cereal_recognition");    // Initialize node name
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);                 // Code will try to run at LOOP_RATE Hz

    RobotPose robot_pose(0,0,0);                     // Index as robot_pose.x, robot_pose.y, robot_pose.phi
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robot_pose);
    ros::ServiceClient move_client = n.serviceClient<nav_msgs::GetPlan>("/make_plan"); // Client for the make_plan service
    
    Boxes boxes;                                    // Initialize box coordinates and templates
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " phi: " 
                  << boxes.coords[i][2] << std::endl;
    }

    ImagePipeline image_pipeline(n);                 // Initialize image object and subscriber.

    std::chrono::time_point<std::chrono::system_clock> start; // Contest count down timer
    start = std::chrono::system_clock::now();

    // std::chrono::time_point<std::chrono::system_clock> camera_start; // Timer for camera to get result
    
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::Quaternion q;
    nav_msgs::GetPlan srv;
    target_pose.header.frame_id = "map";
    srv.request.tolerance = GOAL_TOLERANCE;                   // Tolerance of destination goal in [m]

    // uint64_t camera_seconds_elapsed = 0;
    uint64_t seconds_elapsed = 0;
    uint8_t results[5] = {255, 255, 255, 255, 255};           // 0: Raisin Bran, 1: Cinnamon Toast Crunch, 2: Rice Krispies, 3: Blank
    uint8_t accum = 0;
    uint8_t target = 255;
    uint8_t label = 255;
    uint8_t offset_angle = 0;
    uint8_t yaw_adjust = 0;
    float trajectory_x = 0;
    float trajectory_y = 0;
    float trajectory_yaw = 0;
    float cereal_x = 0;
    float cereal_y = 0;
    float cereal_yaw = 0;
    bool visited[5] = {1, 1, 1, 1, 1};
    bool is_positive = 0;
    bool is_back_at_start = 0;
    bool arrived_at_target = 0;
    

    // Execute strategy.
    while(ros::ok() && seconds_elapsed <= 300 && (!is_back_at_start)) {
        ros::spinOnce();

        if (accum > 0) {
            target = GetNearestNeighbour(robot_pose, boxes.coords, visited); //output uint8_t from 0-4
            if (target > 4) continue;

            cereal_x = boxes.coords[target][0];
            cereal_y = boxes.coords[target][1];
            cereal_yaw = boxes.coords[target][2];

            int i = 0;
            while (offset_angle <= MAX_ANGLE) {

                is_positive = i % 2;
                if (is_positive) {                                      // Odd number for i, positive angle offset
                    offset_angle = ANGLE_INCREMENT * (i + 1)/2;         // Increment of ANGLE_INCREMENT degrees
                    yaw_adjust = (offset_angle * M_PI/180) + cereal_yaw;
                } 
                
                else { //even numbers, include the initial 0 and the sunsequent clockwise locations in negative driection
                    offset_angle = ANGLE_INCREMENT * i/2;
                    yaw_adjust = -(offset_angle * M_PI/180) + cereal_yaw;
                }

                trajectory_x = VISION_RADIUS*cosf(yaw_adjust) + x_coordinate;
                trajectory_y = VISION_RADIUS*sinf(yaw_adjust) + y_coordinate;
                trajectory_phi = (yaw_adjust + M_PI) % (2*M_PI);

                // try to make plan with trajectory_x, trajectory_y, trajectory_phi, if so, break loop
                target_pose.header.stamp = ros::Time::now();
                target_pose.pose.position.x = trajectory_x; // index coordinates file;
                target_pose.pose.position.y = trajectory_y; // index coordinates file;

                q = tf::createQuaternionMsgFromYaw(phiGoal);
                target_pose.pose.orientation.x = 0.0;
                target_pose.pose.orientation.y = 0.0;
                target_pose.pose.orientation.z = q.z;
                target_pose.pose.orientation.w = q.w;

                srv.request.start  = GlobalRobotPose;
                srv.request.goal = target_pose;

                if (move_client.call(srv)) break;
                i++;
            }
        }
        else { // All labels at locations are known
            trajectory_x = 0;
            trajectory_y = 0;
            trajectory_phi = 0;
            is_back_at_start = true;
        }

        arrived_at_target = robot_pose.moveToGoal(trajectory_x, trajectory_y, trajectory_phi);

        if((arrived_at_target) && (!is_back_at_start)) {
            ros::Duration(ARRIVAL_DELAY).sleep();               // Small pause to account for momentum shift
            // camera_start = std::chrono::system_clock::now();
            // camera_seconds_elapsed = 0;
            // label = 255;
            // while ((camera_seconds_elapsed < CAMERA_LIMIT) && (label == 255)) {
            //     label = image_pipeline.getTemplateID(boxes); //int from 0-3
            //     camera_seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-camera_start).count(); //time elapsed trying to read image
            // }

            label = image_pipeline.getTemplateID(boxes); //int from 0-3
            visited[target] = 0;
            results[target] = label;// OpenCV, !!!Ask OpenCV people to return 255 if no good image found. Move onto next goal

            accum = accumulate(visited, visited + 5, accum=0);

            if (accum == 1) {
                if (CheckRepeat(visited)) { //input is visited array, output is boolean
                    results = FillResults(results); // Input is results array, output is results array
                    accum = 0;
                }
            }
        }
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
        loop_rate.sleep();                              // Delay function for 100ms

    }
    return 0;
}
